#include <stdio.h>
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"

typedef struct {
    float x_offset;
    float y_offset;
    float x_width;
    float y_height;
    bool enabled;
} keepout_config_t;

static keepout_config_t config;
static nvs_address_t nvs_addr;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static travel_limits_ptr next_check_travel_limits = NULL;

#define SETTING_X_OFFSET   Setting_UserDefined_0
#define SETTING_Y_OFFSET   Setting_UserDefined_1
#define SETTING_X_WIDTH    Setting_UserDefined_2
#define SETTING_Y_HEIGHT   Setting_UserDefined_3

static status_code_t set_setting(setting_id_t id, float value)
{
    switch(id) {
        case SETTING_X_OFFSET:  config.x_offset = value; break;
        case SETTING_Y_OFFSET:  config.y_offset = value; break;
        case SETTING_X_WIDTH:   config.x_width  = value; break;
        case SETTING_Y_HEIGHT:  config.y_height = value; break;
        default: return Status_Unhandled;
    }

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
    return Status_OK;
}

static float get_setting(setting_id_t id)
{
    switch(id) {
        case SETTING_X_OFFSET:  return config.x_offset;
        case SETTING_Y_OFFSET:  return config.y_offset;
        case SETTING_X_WIDTH:   return config.x_width;
        case SETTING_Y_HEIGHT:  return config.y_height;
        default: return 0.0f;
    }
}

static const setting_detail_t plugin_settings[] = {
    { SETTING_X_OFFSET, Group_UserSettings, "Keepout X Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_OFFSET, Group_UserSettings, "Keepout Y Offset", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_X_WIDTH,  Group_UserSettings, "Keepout X Width",  "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
    { SETTING_Y_HEIGHT, Group_UserSettings, "Keepout Y Height", "mm", Format_Decimal, "#0.00", NULL, NULL, Setting_IsLegacyFn, set_setting, get_setting },
};

static bool travel_limits_check(float *target, axes_signals_t axes, bool is_cartesian)
{

    float x = target[X_AXIS];
    float y = target[Y_AXIS];



    if (config.enabled) {
        float xmin = -config.x_offset - config.x_width;
        float xmax = -config.x_offset;
        float ymin = -config.y_offset - config.y_height;
        float ymax = -config.y_offset;

        snprintf(buf, sizeof(buf),
            "Keepout zone: X[%.2f..%.2f] Y[%.2f..%.2f]", xmin, xmax, ymin, ymax);
        report_message(buf, Message_Info);

        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) {
            report_message("Keepout triggered!", Message_Info);
            return false;
        }
    }

    // Chain to original function
    return next_check_travel_limits ? next_check_travel_limits(target, axes, is_cartesian) : true;
}

static user_mcode_type_t mcode_check(user_mcode_t mcode)
{
    return (mcode == 810 || mcode == 811) ? UserMCode_Normal :
           (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate(parser_block_t *gc_block)
{
    return Status_OK;
}

static void mcode_execute(uint_fast16_t state, parser_block_t *gc_block)
{
    if(state == STATE_CHECK_MODE)
        return;

    if(gc_block->user_mcode == 810) {
        config.enabled = true;
        report_message("Keepout ENABLED", Message_Info);

        char region[100];
        float xmin = -config.x_offset - config.x_width;
        float xmax = -config.x_offset;
        float ymin = -config.y_offset - config.y_height;
        float ymax = -config.y_offset;
        snprintf(region, sizeof(region), "Keepout Region: X[%.2f..%.2f] Y[%.2f..%.2f]", xmin, xmax, ymin, ymax);
        report_message(region, Message_Info);
    } else if(gc_block->user_mcode == 811) {
        config.enabled = false;
        report_message("Keepout DISABLED", Message_Info);
    }
}

static void keepout_restore(void)
{
    config.x_offset = 50.0f;
    config.y_offset = 50.0f;
    config.x_width  = 40.0f;
    config.y_height = 40.0f;
    config.enabled  = true;

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void keepout_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        keepout_restore();
}

static void onReportOptions(bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin("SIENCI Keepout Zone", "0.1");
}

void grblhal_keepout_plugin_init(void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = keepout_load,
        .restore = keepout_restore
    };

    if((nvs_addr = nvs_alloc(sizeof(config)))) {
        keepout_load();


        // Subscribe to option reporting
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        // Subscribe M-codes
        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        settings_register(&settings);

        // Register travel limits filter
        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = travel_limits_check;

        report_message("Keepout plugin initialized", Message_Info);
    }
}
