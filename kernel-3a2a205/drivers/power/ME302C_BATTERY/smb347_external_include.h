#ifndef _SMB347_EXTERNAL_INCLUDE_H
#define _SMB347_EXTERNAL_INCLUDE_H 1

typedef enum {
    USB_IN,
    AC_IN,
    CABLE_OUT,
    ENABLE_5V,
    DISABLE_5V,
};

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB347Charger(int usb_state);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb347_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb347_charging_toggle(bool on);

/* To know if charger has an error
 *
 * return true means charger has an error */
bool smb347_has_charger_error(void);

#define LG "1"
#define COSLIGHT "0"
#define SMB345_DEV_NAME "smb345"

#endif /* _SMB347_EXTERNAL_INCLUDE_H */

