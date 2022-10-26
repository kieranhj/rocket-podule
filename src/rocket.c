#ifdef WIN32
#include <windows.h>
#endif

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "rocket.h"
#include "podule_api.h"
#include <sync.h>

#ifdef WIN32
extern __declspec(dllexport) const podule_header_t *podule_probe(const podule_callbacks_t *callbacks, char *path);
#else
#define BOOL int
#define APIENTRY
#endif

static const podule_callbacks_t *podule_callbacks;
char podule_path[512];

#ifdef DEBUG_LOG
static FILE *rocket_logf;
#endif

enum
{
        ID_PREFIX,
        ID_SPEED,
        ID_PATH,
        ID_LOAD
};

void rocket_log(const char *format, ...)
{
#ifdef DEBUG_LOG
   char buf[1024];
//return;
        printf("Made rocket log!\n");
   if (!rocket_logf) rocket_logf=fopen("rocket_log.txt","wt");
   va_list ap;
   va_start(ap, format);
   vsprintf(buf, format, ap);
   va_end(ap);
   fputs(buf,rocket_logf);
   fflush(rocket_logf);
#endif
}

typedef struct rocket_podule_t
{
        podule_t *podule;

        struct sync_device *device;
        struct sync_cb cb;

        int audio_is_playing;
        int vsyncs;
        int vpr;        /* vsyncs per row */

        int num_tracks;
        const struct sync_track* s_tracks[];
} rocket_podule_t;

/*
        Podule address space: &3000000
        MEMC read space: &300Cxxx for Podule #3
        Addr range from 0 to &3FFF
        Read from bottom half word
        Write to top half word
*/
static int val_f_as_fp(float val_f)
{
        int val_sign = val_f < 0.0f ? -1 : 1;
        int val_fp = fabs(val_f) * (1 << 16);
        return val_fp * val_sign; /* fixed point s15:16 format for now */
}

static int track_no_from_addr(uint32_t addr)
{
        return addr >> 3;
}

static float row_from_vsyncs(int vsyncs, int vpr)
{
        return vsyncs / (float)vpr;
}

static uint8_t rocket_read_b(struct podule_t *podule, podule_io_type type, uint32_t addr)
{
        rocket_podule_t *rocketpod = podule->p;

        if (type != PODULE_IO_TYPE_MEMC)
                return 0xff;

        if (addr >= 0x3ff8) {
                return (addr & 4) ? rocketpod->audio_is_playing : ((rocketpod->vsyncs >> (8 * (addr&1))) & 0xff);
        }

        int track_no = track_no_from_addr(addr);

        if (track_no >= rocketpod->num_tracks) {
                return 0;
        }

        float row = row_from_vsyncs(rocketpod->vsyncs, rocketpod->vpr);

        int val_fp = val_f_as_fp(sync_get_val(rocketpod->s_tracks[track_no], row));
        int val_hw = (addr & 4) ? (val_fp >> 16) : (val_fp & 0xffff);

        /* return individual byte */
        return (val_hw >> (8 * (addr&1))) & 0xff;
}

static uint16_t rocket_read_w(struct podule_t *podule, podule_io_type type, uint32_t addr)
{
        rocket_podule_t *rocketpod = podule->p;

        if (type != PODULE_IO_TYPE_MEMC)
                return 0xffff;

        if (addr >= 0x3ff8) {
                return (addr & 4) ? rocketpod->audio_is_playing : rocketpod->vsyncs;
        }

        int track_no = track_no_from_addr(addr);

        if (track_no >= rocketpod->num_tracks) {
                return 0;
        }

        float row = row_from_vsyncs(rocketpod->vsyncs, rocketpod->vpr);
        int val_fp = val_f_as_fp(sync_get_val(rocketpod->s_tracks[track_no], row));

        /* return hi word or lo word */
        return (addr & 4) ? (val_fp >> 16) : (val_fp & 0xffff);
}

static void rocket_write_b(struct podule_t *podule, podule_io_type type, uint32_t addr, uint8_t val)
{
        rocket_podule_t *rocketpod = podule->p;

        if (type != PODULE_IO_TYPE_MEMC)
                return;

        if (addr == 0x3ff8) {
                rocketpod->vsyncs = (rocketpod->vsyncs & 0xff00) | val;
        } else if (addr == 0x3ff9) {
                rocketpod->vsyncs = (val << 8) | (rocketpod->vsyncs & 0xff);
        } else if (addr == 0x3ffc) {
                rocketpod->audio_is_playing = val;
        }
}

static void rocket_write_w(struct podule_t *podule, podule_io_type type, uint32_t addr, uint16_t val)
{
        rocket_podule_t *rocketpod = podule->p;

        if (type != PODULE_IO_TYPE_MEMC)
                return;

        if (addr == 0x3ff8) {
                rocketpod->vsyncs = val;
        } else if (addr == 0x3ffc) {
                rocketpod->audio_is_playing = val;
        }
}

static int rocket_run(struct podule_t *podule, int timeslice_us)
{
        rocket_podule_t *rocketpod = podule->p;

        float row = row_from_vsyncs(rocketpod->vsyncs, rocketpod->vpr);

	if (sync_update(rocketpod->device, row, &rocketpod->cb, podule)) {
		sync_tcp_connect(rocketpod->device, "localhost", SYNC_DEFAULT_PORT);
        }

        return 10*1000;  /* 10ms = 100Hz */
}

static void rocket_sync_pause(void* data, int flag)
{
        rocket_podule_t *rocketpod = ((struct podule_t *)data)->p;

	if (flag)
		rocketpod->audio_is_playing = 0;
	else
		rocketpod->audio_is_playing = 1;
}

static void rocket_sync_set_row(void* data, int row)
{
        rocket_podule_t *rocketpod = ((struct podule_t *)data)->p;
        rocketpod->vsyncs = row * rocketpod->vpr;
}

static int rocket_sync_is_playing(void* data)
{
        rocket_podule_t *rocketpod = ((struct podule_t *)data)->p;
	return rocketpod->audio_is_playing;
}

static void rocket_sync_write_key(void *data, FILE *fp, char type, int row, key_value value)
{
        rocket_podule_t *rocketpod = ((struct podule_t *)data)->p;
        uint32_t time_and_type = (row * rocketpod->vpr) | type << 24;
        int val_fp = val_f_as_fp(value.val);
        fwrite(&time_and_type, sizeof(uint32_t), 1, fp);
        fwrite(&val_fp, sizeof(int), 1, fp);
}

static int rocket_init(struct podule_t *podule)
{
        FILE *f;
        const char *track_list_fn = podule_callbacks->config_get_string(podule, "track_list", "");

        rocket_log("rocket_init: Opening track list '%s'.\n", track_list_fn);

        f = fopen(track_list_fn, "rt");
        if (!f)
        {
                rocket_log("rocket_init: Failed to open track list '%s'.\n", track_list_fn);
                return -1;
        }

        int num_tracks = 0;
        char track_name[256];
        int type = 0;
        while(fscanf(f, "%s %d", track_name, &type) == 2)
        {
                num_tracks++;
        }

        rocket_log("rocket_init: Found %d tracks in list.\n", num_tracks);

        /* extend rocket_podule_t structure by num_tracks sync_track pointers. */
        rocket_podule_t *rocketpod = malloc(sizeof(rocket_podule_t) + num_tracks * sizeof(struct sync_track *));
        memset(rocketpod, 0, sizeof(rocket_podule_t));

        const char *prefix = podule_callbacks->config_get_string(podule, "prefix", "");
	rocket_log("rocket_init: Creating Rocket device with prefix '%s'\n", prefix);
	rocketpod->device = sync_create_device(prefix);
	if (!rocketpod->device) 
	{
		rocket_log("rocket_init: Unable to create Rocket device with prefix '%s'\n", prefix);
		return -1;
	}

	rocketpod->cb.is_playing = rocket_sync_is_playing;
	rocketpod->cb.pause = rocket_sync_pause;
	rocketpod->cb.set_row = rocket_sync_set_row;
        rocketpod->cb.write_key = rocket_sync_write_key;

	rocket_log("rocket_init: Connecting to Rocket device on localhost at port %d\n", SYNC_DEFAULT_PORT);
	if (sync_tcp_connect(rocketpod->device, "localhost", SYNC_DEFAULT_PORT)) 
	{
		rocket_log("rocket_init: Failed to connect to Rocket device on localhost at port %d\n", SYNC_DEFAULT_PORT);
		return -1;
	}

        fseek(f, 0, SEEK_SET);
        int i = 0;
        while(fscanf(f, "%s %d", track_name, &type) == 2)
        {
		rocket_log("rocket_init: Getting track [%d] named '%s' with type %d\n", i, track_name, type);
                rocketpod->s_tracks[i++] = sync_get_track(rocketpod->device, track_name, type);
        }
        fclose(f);
        assert(i == num_tracks);

        rocketpod->num_tracks = num_tracks;
        int speed = atoi(podule_callbacks->config_get_string(podule, "speed", "4"));
        rocketpod->vpr = speed > 0 ? speed : 4;
        rocketpod->podule = podule;
        podule->p = rocketpod;

	rocket_log("rocket_init: Completed init with speed %d vsyncs per row\n", rocketpod->vpr);
        return 0;
}

static void rocket_close(struct podule_t *podule)
{
        rocket_podule_t *rocketpod = podule->p;
        
        sync_destroy_device(rocketpod->device);
        free(rocketpod);
}

static int config_load_track_list(void *window_p, const struct podule_config_item_t *item, void *new_data)
{
        char fn[256];

        if (!podule_callbacks->config_file_selector(window_p, "Please select a track list file",
                        NULL, NULL, NULL, "*.*|*.*", fn, sizeof(fn), CONFIG_FILESEL_LOAD))
        {
                FILE *f = fopen(fn, "rb");
                if (!f)
                        return 0;
                fclose(f);

                podule_callbacks->config_set_current(window_p, ID_PATH, fn);
                return 1;
        }

        return 0;
}

static podule_config_t rocket_config =
{
        .items =
        {
                {
                        .name = "prefix",
                        .description = "Project name:",
                        .type = CONFIG_STRING,
                        .default_string = "",
                        .id = ID_PREFIX
                },
                {
                        .name = "speed",
                        .description = "Speed (vpr):",
                        .type = CONFIG_STRING,
                        .default_string = "4",
                        .id = ID_SPEED
                },
                {
                        .name = "track_list",
                        .description = "Track list:",
                        .type = CONFIG_STRING,
                        .flags = CONFIG_FLAGS_DISABLED,
                        .default_string = "",
                        .id = ID_PATH
                },
                {
                        .description = "Load track list...",
                        .type = CONFIG_BUTTON,
                        .function = config_load_track_list
                },
                {
                        .type = -1
                }
        }
};

static const podule_header_t rocket_podule_header =
{
        .version = PODULE_API_VERSION,
        .flags = PODULE_FLAGS_UNIQUE,
        .short_name = "rocket",
        .name = "Rocket Sync Tracker",
        .functions =
        {
                .init = rocket_init,
                .close = rocket_close,
                .read_b = rocket_read_b,
                .read_w = rocket_read_w,
                .write_b = rocket_write_b,
                .write_w = rocket_write_w,
                .run = rocket_run
        },
        .config = &rocket_config
};

const podule_header_t *podule_probe(const podule_callbacks_t *callbacks, char *path)
{
        podule_callbacks = callbacks;
        strcpy(podule_path, path);
        
        return &rocket_podule_header;
}

#ifdef WIN32
BOOL APIENTRY DllMain (HINSTANCE hInst     /* Library instance handle. */ ,
                       DWORD reason        /* Reason this function is being called. */ ,
                       LPVOID reserved     /* Not used. */ )
{
    switch (reason)
    {
      case DLL_PROCESS_ATTACH:
        break;

      case DLL_PROCESS_DETACH:
        break;

      case DLL_THREAD_ATTACH:
        break;

      case DLL_THREAD_DETACH:
        break;
    }

    /* Returns TRUE on success, FALSE on failure */
    return TRUE;
}
#endif
