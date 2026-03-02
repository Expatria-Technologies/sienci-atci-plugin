#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "grbl/hal.h"
#include "driver.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"
#include "grbl/settings.h"
#include "grbl/plugins.h"

#define KEEPOUT_TOLERANCE 0.5f

#define MSG_INSIDE_ZONE    "ATCI: You are currently inside the keepout zone. Disable keepout before jogging to safety"
#define MSG_BLOCKED_AT_WALL "ATCI: Jog move blocked at keepout boundary."
#define MSG_CROSSING       "ATCI: Move crosses keepout zone"
#define MSG_TARGET_IN_ZONE "ATCI: Target inside region"

typedef union {
    uint8_t value;
    struct {
        uint8_t plugin_enabled : 1;
        uint8_t unused         : 7;
    } bits;
} config_flags_t;

typedef struct {
    float x_min, y_min, x_max, y_max;
    config_flags_t flags;
} atci_config_t;

typedef struct {
    float x_min, y_min, x_max, y_max;
    bool enabled;
} atci_rt_t;

static atci_config_t config;
static atci_rt_t atci;
static nvs_address_t nvs_addr;

static user_mcode_ptrs_t user_mcode = {0};
static on_report_options_ptr on_report_options = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_report_ngc_parameters_ptr on_report_ngc_parameters = NULL;
static on_state_change_ptr on_state_change = NULL;

typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);

static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

#define SETTING_PLUGIN_ENABLE 683
#define SETTING_X_MIN         684
#define SETTING_Y_MIN         685
#define SETTING_X_MAX         686
#define SETTING_Y_MAX         687

/* --- Helpers --- */

static void keepout_set(void)
{
    atci.x_min = fminf(config.x_min, config.x_max);
    atci.x_max = fmaxf(config.x_min, config.x_max);
    atci.y_min = fminf(config.y_min, config.y_max);
    atci.y_max = fmaxf(config.y_min, config.y_max);
}

static bool is_keepout_active(void)
{
    return config.flags.bits.plugin_enabled && atci.enabled;
}

/* --- Geometry --- */

static bool line_intersects_keepout(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0, dy = y1 - y0;
    float t0 = 0.0f, t1 = 1.0f;
    float p[4] = { -dx, dx, -dy, dy };
    float q[4] = {
        x0 - atci.x_min, atci.x_max - x0,
        y0 - atci.y_min, atci.y_max - y0
    };
    for (int i = 0; i < 4; i++) {
        if (p[i] == 0.0f) {
            if (q[i] < 0.0f) return false;
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else              { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }
    return t0 < t1;
}

static bool calculate_clipped_point(const float *start, const float *end, float *clipped)
{
    float x0 = start[X_AXIS], y0 = start[Y_AXIS];
    float x1 = end[X_AXIS],   y1 = end[Y_AXIS];
    float dx = x1 - x0, dy = y1 - y0;
    float t0 = 0.0f, t1 = 1.0f;
    float p[4] = { -dx, dx, -dy, dy };
    float q[4] = {
        x0 - atci.x_min, atci.x_max - x0,
        y0 - atci.y_min, atci.y_max - y0
    };
    for (int i = 0; i < 4; i++) {
        if (p[i] == 0.0f) {
            if (q[i] < 0.0f) return false;
        } else {
            float t = q[i] / p[i];
            if (p[i] < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else              { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }
    if (t0 > 0.0f) {
        memcpy(clipped, end, sizeof(float) * N_AXIS);
        clipped[X_AXIS] = x0 + t0 * dx;
        clipped[Y_AXIS] = y0 + t0 * dy;
        return true;
    }
    return false;
}

/* --- Travel limit hooks --- */

static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (!is_keepout_active())
        return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;

    float *pos = plan_get_position();
    float x0 = pos ? pos[X_AXIS] : 0.0f;
    float y0 = pos ? pos[Y_AXIS] : 0.0f;
    float xt = target[X_AXIS], yt = target[Y_AXIS];

    bool strictly_inside = (x0 > atci.x_min + KEEPOUT_TOLERANCE && x0 < atci.x_max - KEEPOUT_TOLERANCE &&
                            y0 > atci.y_min + KEEPOUT_TOLERANCE && y0 < atci.y_max - KEEPOUT_TOLERANCE);
    bool technically_inside = (x0 >= atci.x_min && x0 <= atci.x_max &&
                               y0 >= atci.y_min && y0 <= atci.y_max);
    bool target_deep_inside = (xt > atci.x_min + KEEPOUT_TOLERANCE && xt < atci.x_max - KEEPOUT_TOLERANCE &&
                               yt > atci.y_min + KEEPOUT_TOLERANCE && yt < atci.y_max - KEEPOUT_TOLERANCE);

    if (target_deep_inside) {
        report_message((strictly_inside || technically_inside) ? MSG_INSIDE_ZONE : MSG_TARGET_IN_ZONE, Message_Warning);
        return false;
    }

    if (line_intersects_keepout(x0, y0, xt, yt)) {
        report_message(technically_inside ? MSG_INSIDE_ZONE : MSG_CROSSING, Message_Warning);
        return false;
    }

    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;
}

static void keepout_apply_travel_limits(float *target, float *position, work_envelope_t *envelope)
{
    if (!is_keepout_active()) {
        if (prev_apply_travel_limits) prev_apply_travel_limits(target, position, envelope);
        return;
    }

    float x0 = position[X_AXIS], y0 = position[Y_AXIS];
    float xt = target[X_AXIS],   yt = target[Y_AXIS];

    bool strictly_inside = (x0 > atci.x_min + KEEPOUT_TOLERANCE && x0 < atci.x_max - KEEPOUT_TOLERANCE &&
                            y0 > atci.y_min + KEEPOUT_TOLERANCE && y0 < atci.y_max - KEEPOUT_TOLERANCE);
    bool technically_inside = (x0 >= atci.x_min && x0 <= atci.x_max &&
                               y0 >= atci.y_min && y0 <= atci.y_max);

    if (strictly_inside) {
        report_message(MSG_INSIDE_ZONE, Message_Warning);
        memcpy(target, position, sizeof(float) * N_AXIS);
        return;
    }

    bool target_deep_inside = (xt > atci.x_min + KEEPOUT_TOLERANCE && xt < atci.x_max - KEEPOUT_TOLERANCE &&
                               yt > atci.y_min + KEEPOUT_TOLERANCE && yt < atci.y_max - KEEPOUT_TOLERANCE);

    if (target_deep_inside || line_intersects_keepout(x0, y0, xt, yt)) {
        float clipped[N_AXIS];
        bool clipped_ok = calculate_clipped_point(position, target, clipped);
        report_message(technically_inside ? MSG_INSIDE_ZONE : MSG_BLOCKED_AT_WALL, Message_Warning);
        memcpy(target, clipped_ok ? clipped : position, sizeof(float) * N_AXIS);
        return;
    }

    if (prev_apply_travel_limits) prev_apply_travel_limits(target, position, envelope);
}

/* --- M960 --- */

static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    if (mcode == 960) return UserMCode_Normal;
    return user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported;
}

static status_code_t mcode_validate(parser_block_t *gc_block)
{
    if (gc_block->user_mcode != 960)
        return user_mcode.validate ? user_mcode.validate(gc_block) : Status_Unhandled;

    if (gc_block->words.p) {
        if (gc_block->values.p != 0.0f && gc_block->values.p != 1.0f)
            return Status_GcodeValueOutOfRange;
        gc_block->words.p = 0;
    }
    return Status_OK;
}

static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if (gc_block->user_mcode != 960) {
        if (user_mcode.execute) user_mcode.execute(state, gc_block);
        return;
    }
    if (state == STATE_CHECK_MODE) return;

    if (gc_block->words.p) {
        atci.enabled = (gc_block->values.p == 1.0f);
        report_message(atci.enabled ? "ATCi keepout enabled." : "ATCi keepout disabled.", Message_Info);
    } else {
        report_message("Use M960 P1 to enable ATCi keepout, M960 P0 to disable.", Message_Info);
    }
}

/* --- Settings --- */

static const setting_detail_t plugin_settings[] = {
    { SETTING_PLUGIN_ENABLE, Group_Limits, "ATCi Plugin",        NULL, Format_XBitfield, "Enable", NULL, NULL, Setting_NonCore, &config.flags.value },
    { SETTING_X_MIN,         Group_Limits, "ATCi Keepout X Min", "mm", Format_Decimal,   "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_min },
    { SETTING_Y_MIN,         Group_Limits, "ATCi Keepout Y Min", "mm", Format_Decimal,   "-####0.00", "-10000", "10000", Setting_NonCore, &config.y_min },
    { SETTING_X_MAX,         Group_Limits, "ATCi Keepout X Max", "mm", Format_Decimal,   "-####0.00", "-10000", "10000", Setting_NonCore, &config.x_max },
    { SETTING_Y_MAX,         Group_Limits, "ATCi Keepout Y Max", "mm", Format_Decimal,   "-####0.00", "-10000", "10000", Setting_NonCore, &config.y_max },
};

static void atci_save(void)
{
    keepout_set();
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void atci_restore(void)
{
    config.x_min = 10.0f; config.y_min = 10.0f;
    config.x_max = 50.0f; config.y_max = 50.0f;
    config.flags.value = 0;
    atci.enabled = false;
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void atci_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        atci_restore();
    keepout_set();
    atci.enabled = false; /* always start disabled; user must send M960 P1 */
}

/* --- Reports --- */

static void onReportOptions(bool newopt)
{
    if (on_report_options) on_report_options(newopt);
    if (!newopt) report_plugin("SIENCI ATCi plugin", "0.5.0");
}

static void onReportNgcParameters(void)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "[ATCI:%.2f,%.2f,%.2f,%.2f]" ASCII_EOL,
             atci.x_max, atci.x_min, atci.y_max, atci.y_min);
    hal.stream.write(buf);
    if (on_report_ngc_parameters) on_report_ngc_parameters();
}

static void onStateChange(uint_fast16_t state)
{
    static uint_fast16_t prev_state = STATE_IDLE;

    if (state == STATE_TOOL_CHANGE)
        atci.enabled = false;
    else if (prev_state == STATE_TOOL_CHANGE)
        atci.enabled = true;  /* auto-rearm when TC completes */

    prev_state = state;
    if (on_state_change)
        on_state_change(state);
}

static void onRealtimeReport(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "|ATCI:%s", atci.enabled ? "E" : "");
    stream_write(buf);
    if (on_realtime_report) on_realtime_report(stream_write, report);
}

/* --- Init --- */

void atci_init(void)
{
    static setting_details_t settings = {
        .settings   = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load       = atci_load,
        .save       = atci_save,
        .restore    = atci_restore
    };

    if (!(nvs_addr = nvs_alloc(sizeof(config)))) return;

    atci_load();

    prev_check_travel_limits  = grbl.check_travel_limits;
    grbl.check_travel_limits  = travel_limits_check;
    prev_apply_travel_limits  = grbl.apply_travel_limits;
    grbl.apply_travel_limits  = keepout_apply_travel_limits;

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check    = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute  = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    on_report_ngc_parameters = grbl.on_report_ngc_parameters;
    grbl.on_report_ngc_parameters = onReportNgcParameters;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChange;

    settings_register(&settings);

    report_message("Sienci ATCi plugin v0.5.0 initialized", Message_Info);
}