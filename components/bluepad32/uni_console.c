/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2022 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "uni_console.h"

#include <argtable3/argtable3.h>
#include <cmd_nvs.h>
#include <cmd_system.h>
#include <esp_console.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "sdkconfig.h"
#include "uni_bt_setup.h"
#include "uni_debug.h"
#include "uni_hid_device.h"
#include "uni_mouse_quadrature.h"
#include "uni_platform_unijoysticle.h"

#ifdef CONFIG_ESP_CONSOLE_USB_CDC
#error This example is incompatible with USB CDC console. Please try "console_usb" example instead.
#endif  // CONFIG_ESP_CONSOLE_USB_CDC

static const char* TAG = "console";
#define PROMPT_STR "bp32"

static struct {
    struct arg_dbl* value;
    struct arg_end* end;
} mouse_set_args;

static struct {
    struct arg_int* value;
    struct arg_end* end;
} set_gap_security_level_args;

static struct {
    struct arg_int* max;
    struct arg_int* min;
    struct arg_int* len;
    struct arg_end* end;
} set_gap_periodic_inquiry_args;

static int devices_info(int argc, char** argv) {
    /* FIXME: Should be called from btstack thread */
    uni_hid_device_dump_all();
    return 0;
}

static int mouse_get(int argc, char** argv) {
    char buf[32];
    float scale = uni_mouse_quadrature_get_scale_factor();

    // ets_printf() doesn't support "%f"
    sprintf(buf, "global mouse scale = %f\n", scale);
    logi(buf);
    return 0;
}

static int mouse_set(int argc, char** argv) {
    float scale;

    int nerrors = arg_parse(argc, argv, (void**)&mouse_set_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mouse_set_args.end, argv[0]);
        return 1;
    }

    scale = mouse_set_args.value->dval[0];
    uni_mouse_quadrature_set_scale_factor(scale);
    logi("Done\n");
    return 0;
}

static int set_gap_security_level(int argc, char** argv) {
    int gap;

    int nerrors = arg_parse(argc, argv, (void**)&set_gap_security_level_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_gap_security_level_args.end, argv[0]);
        return 1;
    }

    gap = set_gap_security_level_args.value->ival[0];
    uni_bt_setup_set_gap_security_level(gap);
    logi("Done. Restart required. Type 'restart' + Enter\n");
    return 0;
}

static int get_gap_security_level(int argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int gap = uni_bt_setup_get_gap_security_level();
    logi("GAP security level: %d\n", gap);
    return 0;
}

static int set_gap_periodic_inquiry(int argc, char** argv) {
    int min, max, len;

    int nerrors = arg_parse(argc, argv, (void**)&set_gap_periodic_inquiry_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_gap_periodic_inquiry_args.end, argv[0]);
        return 1;
    }

    max = set_gap_periodic_inquiry_args.max->ival[0];
    min = set_gap_periodic_inquiry_args.min->ival[0];
    len = set_gap_periodic_inquiry_args.len->ival[0];
    uni_bt_setup_set_gap_max_peridic_length(max);
    uni_bt_setup_set_gap_min_peridic_length(min);
    uni_bt_setup_set_gap_inquiry_length(len);
    logi("Done. Restart required. Type 'restart' + Enter\n");
    return 0;
}

static int get_gap_periodic_inquiry(int argc, char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int max = uni_bt_setup_get_gap_max_periodic_lenght();
    int min = uni_bt_setup_get_gap_min_periodic_lenght();
    int len = uni_bt_setup_get_gap_inquiry_lenght();
    logi("GAP max periodic len: %d, min periodic len: %d, inquiry len: %d\n", max, min, len);
    return 0;
}

static void register_bluepad32() {
    mouse_set_args.value = arg_dbl1(NULL, NULL, "<value>", "Global mouse scale factor. Higher means slower");
    mouse_set_args.end = arg_end(2);

    set_gap_security_level_args.value = arg_int1(NULL, NULL, "<value>", "GAP security level");
    set_gap_security_level_args.end = arg_end(2);

    set_gap_periodic_inquiry_args.max = arg_int1(NULL, NULL, "<max>", "Max periodic length. Must be bigger than <min>");
    set_gap_periodic_inquiry_args.min = arg_int1(NULL, NULL, "<min>", "Min periodic length. Must be less than <max>");
    set_gap_periodic_inquiry_args.len = arg_int1(NULL, NULL, "<len>", "Inquiry length. Must be less than <min>");
    set_gap_periodic_inquiry_args.end = arg_end(4);

    const esp_console_cmd_t cmd_devices = {
        .command = "devices",
        .help = "Get information about connected devices",
        .hint = NULL,
        .func = &devices_info,
    };

    const esp_console_cmd_t cmd_mouse_get = {
        .command = "get_mouse_scale",
        .help = "Get global mouse scale factor",
        .hint = NULL,
        .func = &mouse_get,
    };

    const esp_console_cmd_t cmd_mouse_set = {
        .command = "set_mouse_scale",
        .help =
            "Set global mouse scale factor. Default: 1.0\n"
            "  Example: set_mouse_scale 1.52",
        .hint = NULL,
        .func = &mouse_set,
        .argtable = &mouse_set_args,
    };

    const esp_console_cmd_t cmd_set_gap_security_level = {
        .command = "set_gap_security_level",
        .help =
            "Set GAP security level. Default: 2\n"
            "  Recommended values: 0, 1 or 2",
        .hint = NULL,
        .func = &set_gap_security_level,
        .argtable = &set_gap_security_level_args,
    };

    const esp_console_cmd_t cmd_get_gap_security_level = {
        .command = "get_gap_security_level",
        .help = "Get GAP security level",
        .hint = NULL,
        .func = &get_gap_security_level,
    };

    const esp_console_cmd_t cmd_set_gap_periodic_inquiry = {
        .command = "set_gap_periodic_inquiry",
        .help =
            "Set GAP periodic inquiry mode. Default: 5 4 3.\n"
            "  Used for new connections / reconnections.\n"
            "  1 unit == 1.28 seconds\n"
            "  See Section 7.1.3 'Periodic Inquiry Mode Command' from Bluetooth spec",
        .hint = NULL,
        .func = &set_gap_periodic_inquiry,
        .argtable = &set_gap_periodic_inquiry_args,
    };

    const esp_console_cmd_t cmd_get_gap_periodic_inquiry = {
        .command = "get_gap_periodic_inquiry",
        .help = "Get GAP periodic inquiry parameters",
        .hint = NULL,
        .func = &get_gap_periodic_inquiry,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_devices));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_mouse_get));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_mouse_set));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_set_gap_security_level));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_get_gap_security_level));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_set_gap_periodic_inquiry));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_get_gap_periodic_inquiry));
}

void uni_console_init(void) {
    esp_console_repl_t* repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = 80;  // CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

#if CONFIG_CONSOLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    /* Register commands */
    esp_console_register_help_command();
    register_system_common();
#if CONFIG_BLUEPAD32_CONSOLE_NVS_COMMAND_ENABLE
    register_nvs();
#endif  // CONFIG_BLUEPAD32_CONSOLE_NVS_COMMAND_ENABLE

    register_bluepad32();
#if CONFIG_BLUEPAD32_PLATFORM_UNIJOYSTICLE
    uni_platform_unijoysticle_register_cmds();
#endif  // CONFIG_BLUEPAD32_PLATFORM_UNIJOYSTICLE

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // Should not happen
    // vTaskDelete(NULL);
}