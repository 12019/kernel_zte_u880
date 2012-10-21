#ifndef __WLAN_H__
#define __WLAN_H__

#define WB_STATE_STOPPED  (0) // module removed
#define WB_STATE_OPENING  (1) // module inserting
#define WB_STATE_RUNNING  (2) // module running
#define WB_STATE_CLOSING  (3) // module removing

struct wlan_platform_data
{
    int wlan_state;
    int bt_state;
    void (*chip_gpio_setpower)(int);
};

struct wlan_find_info
{
    const char *name;
    int id;
    void *dev;
};
#endif
