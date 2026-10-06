#include "shot_usb.h"
#include "capture.h"
#include "shot_store.h"
#include "usb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool shot_usb_command(const char *command)
{
    if (!strcmp(command, "SHOTSTATUS")) {
        const shot_manifest_t *m = shot_store_manifest();
        char id[33] = "";
        if (shot_store_saved()) for (unsigned i = 0; i < 16; ++i) snprintf(id + 2 * i, 3, "%02x", m->id[i]);
        usb_printf("SHOT state=%s available=%d saved=%d error=%d id=%s quality=%u saturation=%u\n",
            capture_shot_state(), shot_store_available(), shot_store_saved(), capture_shot_error(), id,
            shot_store_saved() ? (unsigned)m->quality_mask : 0,
            shot_store_saved() ? (unsigned)m->saturation_mask : 0);
    } else if (!strncmp(command, "SHOTARM ", 8)) {
        unsigned pre, post, threshold; int consumed = 0;
        bool parsed = sscanf(command + 8, "%u %u %u %n", &pre, &post, &threshold, &consumed) == 3 && consumed > 0;
        const char *metadata = command + 8 + consumed;
        bool ok = parsed && (!metadata[0] || metadata[0] == '{') &&
            capture_start_shot(pre, post, threshold, metadata[0] ? metadata : "{\"source\":\"USB\"}");
        usb_printf("%s\n", ok ? "SHOT ARMED" : "ERR shot_arm_failed");
    } else if (!strcmp(command, "SHOTTRIGGER")) {
        usb_printf("%s\n", capture_trigger_shot() ? "SHOT TRIGGERED" : "ERR wait_for_pre_window_or_arm");
    } else if (!strcmp(command, "SHOTCANCEL")) {
        bool active = capture_shot_active();
        if (active) capture_stop();
        usb_printf("%s\n", active ? "SHOT CANCEL REQUESTED" : "ERR no_active_shot");
    } else if (!strcmp(command, "SHOTDELETE")) {
        bool ok = !capture_running() && shot_store_delete() == ESP_OK;
        usb_printf("%s\n", ok ? "SHOT DELETED" : "ERR busy_or_storage_unavailable");
    } else if (!strcmp(command, "SHOTGET")) {
        if (capture_running() || !shot_store_saved()) usb_printf("ERR no_idle_saved_shot\n");
        else {
            uint8_t *page = malloc(SHOT_PAGE_BYTES);
            if (!page) usb_printf("ERR no_memory\n");
            else {
                const shot_manifest_t *m = shot_store_manifest();
                usb_printf("SHOT DATA bytes=%u\n", (unsigned)(sizeof(*m) + (m->end_seq - m->first_seq) * SHOT_PAGE_BYTES));
                bool ok = usb_send(m, sizeof(*m));
                for (uint32_t seq = m->first_seq; ok && seq < m->end_seq; ++seq)
                    ok = shot_store_read_page(seq, page) == ESP_OK && usb_send(page, SHOT_PAGE_BYTES);
                free(page);
                /* Failure yields a truncated download; never inject text into it. */
            }
        }
    } else return false;
    return true;
}
