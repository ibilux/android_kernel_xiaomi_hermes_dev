/*
 * Copyright (C) 2015 Xiaomi, Inc.
 *
 * Authors: ChenGang <ben.chen@cellwise-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.And this driver depends on
 * I2C and uses IIC bus for communication with the host.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include <mach/charging.h>
#include <mach/mt_gpio.h>

#define MODE_SLEEP         (0x3<<6)
#define MODE_NORMAL        (0x0<<6)
#define CONFIG_UPDATE_FLG  (0x1<<1)
#define ATHD               (0x0<<3)

int cw2015_capacity, cw2015_vol, cw2015_battery_type;

static int PowerResetFlag = -1;
static int alg_run_flag = -1;
static int count_num, count_sp;

extern int cw2015_charging_type, cw2015_charging_status;
extern char* saved_command_line;

static u8 config_info_cos[64] = {
    0x17, 0xF3, 0x63, 0x6A, 0x6A, 0x68, 0x68, 0x65, 0x63, 0x60,
    0x5B, 0x59, 0x65, 0x5B, 0x46, 0x41, 0x36, 0x31, 0x28, 0x27,
    0x31, 0x35, 0x43, 0x51, 0x1C, 0x3B, 0x0B, 0x85, 0x22, 0x42,
    0x5B, 0x82, 0x99, 0x92, 0x98, 0x96, 0x3D, 0x1A, 0x66, 0x45,
    0x0B, 0x29, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82, 0x8C,
    0x92, 0x96, 0x54, 0xC2, 0xBA, 0xCB, 0x2F, 0x7D, 0x72, 0xA5,
    0xB5, 0xC1, 0xA5, 0x49
};

static u8 config_info_des[64] = {
    0x17, 0xF9, 0x6D, 0x6D, 0x6B, 0x67, 0x65, 0x64, 0x58, 0x6D,
    0x6D, 0x48, 0x57, 0x5D, 0x4A, 0x43, 0x37, 0x31, 0x2B, 0x20,
    0x24, 0x35, 0x44, 0x55, 0x20, 0x37, 0x0B, 0x85, 0x2A, 0x4A,
    0x56, 0x68, 0x74, 0x6B, 0x6D, 0x6E, 0x3C, 0x1A, 0x5C, 0x45,
    0x0B, 0x30, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52, 0x82, 0x8C,
    0x92, 0x96, 0x64, 0xB4, 0xDB, 0xCB, 0x2F, 0x7D, 0x72, 0xA5,
    0xB5, 0xC1, 0xA5, 0x42
};

static void cw_get_battery_version(void)
{
    cw2015_battery_type = simple_strtol(strstr(saved_command_line, "batversion=") + 11, 0, 10);
    if (cw2015_battery_type == 1)
        printk("[CW2015] battery_type = COS\n");
    else if (cw2015_battery_type == 2)
        printk("[CW2015] battery_type = DES\n");
    else
        printk("[CW2015] battery_type = NULL\n");
}

struct cw_bat_platform_data {
    u8* cw_bat_config_info;
};

static struct cw_bat_platform_data cw_bat_platdata = {
    .cw_bat_config_info = config_info_cos
};

struct cw_battery {
    struct i2c_client *client;
    struct workqueue_struct *battery_workqueue;
    struct delayed_work battery_delay_work;
    struct cw_bat_platform_data *plat_data;

    long sleep_time_capacity_change;
    long run_time_capacity_change;

    long sleep_time_charge_start;
    long run_time_charge_start;

    int usb_online;
    int bat_change;
    int charger_mode;
    int capacity;
    int voltage;
};

struct cw_store {
    long bts;
    int OldSOC;
    int DetSOC;
    int AlRunFlag;
};

static unsigned int cw_convertData(struct cw_battery *cw_bat, unsigned int ts)
{
    unsigned int i = ts % 4096, n = ts / 4096, ret = 6553;

    if (i >= 1700) {
        i -= 1700;
        ret = (ret * 3) / 4;
    }
    if (i >= 1700) {
        i -= 1700;
        ret = (ret * 3) / 4;
    }
    if (i >= 789) {
        i -= 789;
        ret = (ret * 7) / 8;
    }
    if (i >= 381) {
        i -= 381;
        ret = (ret * 15) / 16;
    }
    if (i >= 188) {
        i -= 188;
        ret = (ret * 31) / 32;
    }
    if (i >= 188) {
        i -= 188;
        ret = (ret * 31) / 32;
    }
    if (i >= 93) {
        i -= 93;
        ret = (ret * 61) / 64;
    }
    if (i >= 46) {
        i -= 46;
        ret = (ret * 127) / 128;
    }
    if (i >= 23) {
        i -= 23;
        ret = (ret * 255) / 256;
    }
    if (i >= 11) {
        i -= 11;
        ret = (ret * 511) / 512;
    }
    if (i >= 6) {
        i -= 6;
        ret = (ret * 1023) / 1024;
    }
    if (i >= 3) {
        i -= 3;
        ret = (ret * 2047) / 2048;
    }
    if (i >= 3) {
        i -= 3;
        ret = (ret * 2047) / 2048;
    }

    return ret>>n;
}

static int AlgNeed(struct cw_battery *cw_bat, int dSOC, int SOCT0)
{
    int dSOC1 = dSOC * 100;
    int SOCT01 = SOCT0 * 100;
    if (SOCT01 < 2400) {
        if ((dSOC1 > -100) && (dSOC1 < ((2400 + SOCT01) / 3)))
            return 1;
    } else if (SOCT01 < 4800) {
        if ((dSOC1 > -100) && (dSOC1 < ((7200 - SOCT01) / 3)))
            return 1;
    } else {
        if ((dSOC1 > -100) && (dSOC1 < 800))
            return 1;
    }

    return -1;
}

static int cw_algorithm(struct cw_battery *cw_bat,int real_capacity)
{
    struct file *file = NULL;
    struct cw_store cw;
    struct inode *inode;
    mm_segment_t old_fs;

    unsigned int utemp, utemp1, file_size;
    long timeNow;
    int vmSOC;

    count_num = count_num + 1;
    if (count_num < count_sp)
        return real_capacity;
    else
        count_num = 0;

    timeNow = get_seconds();
    vmSOC = real_capacity;
    if (file == NULL)
        file = filp_open("/data/lastsoc", O_RDWR|O_CREAT, 0644);

    if (IS_ERR(file)) {
        printk("[CW2015] Using real_capacity, because %s doesn't exist\n", "/data/lastsoc");
        return real_capacity;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    inode = file->f_dentry->d_inode;
    file_size = inode->i_size;
    if (file_size < sizeof(cw)) {
        set_fs(old_fs);
        filp_close(file, NULL);
        file = NULL;
        return real_capacity;
    } else {
        file->f_pos = 0;
        vfs_read(file, (char*)&cw, sizeof(cw), &file->f_pos);
    }

    if (PowerResetFlag == 1) {
        PowerResetFlag = -1;
        cw.DetSOC = cw.OldSOC - real_capacity;

        if (AlgNeed(cw_bat, cw.DetSOC, cw.OldSOC))
            cw.AlRunFlag = 1;
        else
            cw.AlRunFlag = -1;

        if ((real_capacity < 100) && (real_capacity > 94)) {
            cw.AlRunFlag = 1;
            vmSOC = 100;
            cw.DetSOC = 100 - real_capacity;
        }
    } else if ((cw.AlRunFlag == 1) && (cw.DetSOC != 0)) {
        utemp1 = 32768 / cw.DetSOC;
        if (cw.bts < timeNow)
            utemp = cw_convertData(cw_bat, (timeNow - cw.bts));
        else
            utemp = cw_convertData(cw_bat, 1);

        if (cw.DetSOC < 0)
            vmSOC = real_capacity - (int)((((unsigned int)(cw.DetSOC * -1) * utemp) + utemp1) / 65536);
        else
            vmSOC = real_capacity + (int)((((unsigned int)cw.DetSOC * utemp) + utemp1) / 65536);

        if (vmSOC == real_capacity)
            cw.AlRunFlag = -1;
    } else {
        count_sp = 10;
        cw.AlRunFlag = -1;
        cw.bts = timeNow;
    }

    alg_run_flag = cw.AlRunFlag;
    if (vmSOC > 100)
        vmSOC = 100;
    else if (vmSOC < 0)
        vmSOC = 0;

    cw.OldSOC = vmSOC;
    file->f_pos = 0;
    vfs_write(file, (char*)&cw, sizeof(cw), &file->f_pos);
    set_fs(old_fs);
    filp_close(file,NULL);
    file = NULL;

    return vmSOC;
}

static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
    int ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        return ret;
    else {
        buf[0] = ret;
        ret = 0;
    }

    return ret;
}

static int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
    int ret = i2c_smbus_write_byte_data(client, reg, buf[0]);

    return ret;
}

static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
    unsigned int data = i2c_smbus_read_word_data(client, reg);
    buf[0] = data & 0x00FF;
    buf[1] = (data & 0xFF00)>>8;

    return 0;
}

static int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret, i;
    u8 reg_val, reset_val;

    /* make sure no in sleep mode */
    ret = cw_read(cw_bat->client, 10, &reg_val);
    if (ret < 0)
        return ret;

    reset_val = reg_val;
    if ((reg_val & MODE_SLEEP) == MODE_SLEEP) {
        printk("[CW2015] Can't update battery info, because device is in sleep mode\n");
        return -1;
    }

    /* update battery info */
    for (i = 0; i < 64; i++) {
        ret = cw_write(cw_bat->client, 16 + i, &(cw_bat->plat_data->cw_bat_config_info[i]));
        if (ret < 0)
            return ret;
    }

    /* check update */
    for (i = 0; i < 64; i++) {
        ret = cw_read(cw_bat->client, 16 + i, &reg_val);
        if (reg_val != cw_bat->plat_data->cw_bat_config_info[i])
            return -1;
    }

    /* tell cw2015 to use new battery info */
    ret = cw_read(cw_bat->client, 8, &reg_val);
    if (ret < 0)
        return ret;

    reg_val |= CONFIG_UPDATE_FLG;
    reg_val &= 0x07; /* clear ATHD */
    reg_val |= ATHD; /* set ATHD */
    ret = cw_write(cw_bat->client, 8, &reg_val);
    if (ret < 0)
        return ret;

    /* check cw2015 for ATHD & update_flag */
    ret = cw_read(cw_bat->client, 8, &reg_val);
    if (ret < 0)
        return ret;

    /* reset */
    reset_val &= ~(0xf<<0); /* MODE_RESTART */
    reg_val = reset_val | (0xf<<0); /* MODE_RESTART */
    ret = cw_write(cw_bat->client, 10, &reg_val);
    if (ret < 0)
        return ret;

    msleep(10);

    ret = cw_write(cw_bat->client, 10, &reset_val);
    if (ret < 0)
        return ret;

    PowerResetFlag = 1;
    msleep(10);

    return 0;
}

static int cw_init(struct cw_battery *cw_bat)
{
    int ret, i;
    u8 reg_val = MODE_SLEEP;

    cw_get_battery_version();
    switch (cw2015_battery_type) {
        case 1:
            cw_bat->plat_data->cw_bat_config_info = config_info_cos;
            break;
        case 2:
            cw_bat->plat_data->cw_bat_config_info = config_info_des;
            break;
        default:
            printk("[CW2015] Can't get battery type, fallback to default (COS)\n");
            cw_bat->plat_data->cw_bat_config_info = config_info_cos;
    }

    if ((reg_val & MODE_SLEEP) == MODE_SLEEP) { /* MODE_SLEEP */
        reg_val = MODE_NORMAL;

        ret = cw_write(cw_bat->client, 10, &reg_val);
        if (ret < 0)
            return ret;
    }

    ret = cw_read(cw_bat->client, 8, &reg_val);
    if (ret < 0)
        return ret;

    if ((reg_val & 0xf8) != ATHD) {
        reg_val &= 0x07; /* clear ATHD */
        reg_val |= ATHD; /* set ATHD */
        ret = cw_write(cw_bat->client, 8, &reg_val);
        if (ret < 0)
            return ret;
    }

    ret = cw_read(cw_bat->client, 8, &reg_val);
    if (ret < 0)
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
        ret = cw_update_config_info(cw_bat);
        if (ret < 0)
            return ret;
    } else {
        for (i = 0; i < 64; i++) {
            ret = cw_read(cw_bat->client, (16 + i), &reg_val);
            if (ret < 0)
                return ret;

            if (cw_bat->plat_data->cw_bat_config_info[i] != reg_val)
                break;
        }

        if (i != 64) {
            printk("[CW2015] Can't update battery info\n");
            ret = cw_update_config_info(cw_bat);
            if (ret < 0)
                return ret;
        }
    }

    for (i = 0; i < 30; i++) {
        ret = cw_read(cw_bat->client, 4, &reg_val);
        if (ret < 0)
            return ret;
        else if (reg_val <= 0x64)
            break;

        msleep(100);
        if (i > 25)
            printk("[CW2015] Input power error\n");
    }

    if (i >= 30) {
        reg_val = MODE_SLEEP;
        ret = cw_write(cw_bat->client, 10, &reg_val);
        printk("[CW2015] Input power error\n");
        return -1;
    }

    return 0;
}

static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_charge_start = new_run_time;
    cw_bat->sleep_time_charge_start = new_sleep_time;
}

static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_capacity_change = new_run_time;
    cw_bat->sleep_time_capacity_change = new_sleep_time;
}

static int rk_usb_update_online(struct cw_battery *cw_bat)
{
    if (cw2015_charging_status) {
        if (cw2015_charging_type == STANDARD_HOST) {
            cw_bat->charger_mode = 1;
            if (cw_bat->usb_online != 1) {
                cw_bat->usb_online = 1;
                cw_update_time_member_charge_start(cw_bat);
            }
            return 0;
        } else {
            cw_bat->charger_mode = 2;
            if (cw_bat->usb_online != 1) {
                cw_bat->usb_online = 1;
                cw_update_time_member_charge_start(cw_bat);
            }
            return 0;
        }
    } else {
        cw_bat->charger_mode = cw2015_charging_status;

        if (cw_bat->usb_online == 0)
            return 0;
        else {
            cw_update_time_member_charge_start(cw_bat);
            cw_bat->usb_online = 0;
            return 1;
        }
    }
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int cw_capacity, allow_change, allow_capacity, charge_time;
    static int if_quickstart,jump_flag, reset_loop;
    long new_run_time, new_sleep_time, capacity_or_aconline_time;
    u8 reg_val[2], reset_val;

    int ret = cw_read_word(cw_bat->client, 4, reg_val);
    if (ret < 0)
        return ret;

    cw_capacity = reg_val[0];
    if ((cw_capacity < 0) || (cw_capacity > 100)) {
        printk("[CW2015] Capacity error, capacity = %d\n", cw_capacity);

        reset_loop++;
        if (reset_loop > 5) {
            reset_val = MODE_SLEEP;
            ret = cw_write(cw_bat->client, 10, &reset_val);
            if (ret < 0)
                return ret;

            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_write(cw_bat->client, 10, &reset_val);
            if (ret < 0)
                return ret;

            ret = cw_init(cw_bat);
            if (ret)
                return ret;

            reset_loop = 0;
        }

        return cw_bat->capacity;
    } else
        reset_loop = 0;

    cw_capacity = cw_algorithm(cw_bat, cw_capacity);

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9)))
        || ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {
        if (!(cw_capacity == 0 && cw_bat->capacity <= 2))
                cw_capacity = cw_bat->capacity;
    }

    if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {
        capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
        capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
        allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / 420;
        if (allow_change > 0) {
            allow_capacity = cw_bat->capacity + allow_change;
            cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
            jump_flag = 1;
        } else if (cw_capacity <= cw_bat->capacity)
            cw_capacity = cw_bat->capacity;
    } else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity) && (cw_capacity >= 90) && (jump_flag == 1)) {
        capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
        capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
        allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / 60;
        if (allow_change > 0) {
            allow_capacity = cw_bat->capacity - allow_change;
            if (cw_capacity >= allow_capacity)
                jump_flag =0;
            else
                cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
        } else if (cw_capacity <= cw_bat->capacity)
            cw_capacity = cw_bat->capacity;
    }

    if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {
        allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / 30);
        allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / 1800);
        allow_capacity = cw_bat->capacity - allow_change;
        cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;

        reset_val = MODE_SLEEP;
        ret = cw_write(cw_bat->client, 10, &reset_val);
        if (ret < 0)
            return ret;

        reset_val = MODE_NORMAL;
        msleep(10);
        ret = cw_write(cw_bat->client, 10, &reset_val);
        if (ret < 0)
            return ret;

        ret = cw_init(cw_bat);
        if (ret)
            return ret;
    }

    if ((cw_bat->charger_mode > 0) && (cw_capacity == 0)) {
        charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
        if ((charge_time > 1800) && (if_quickstart == 0)) {
            reset_val = MODE_SLEEP;
            ret = cw_write(cw_bat->client, 10, &reset_val);
            if (ret < 0)
                return ret;

            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_write(cw_bat->client, 10, &reset_val);
            if (ret < 0)
                return ret;

            ret = cw_init(cw_bat);
            if (ret)
                return ret;

            if_quickstart = 1;
        }
    } else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0))
        if_quickstart = 0;

    return cw_capacity;
}

static int cw_get_vol(struct cw_battery *cw_bat)
{
    u8 reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage, ret;

    ret = cw_read_word(cw_bat->client, 2, reg_val);
    if (ret < 0)
        return ret;
    value16 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, 2, reg_val);
    if (ret < 0)
        return ret;
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, 2, reg_val);
    if (ret < 0)
        return ret;
    value16_2 = (reg_val[0] << 8) + reg_val[1];

    if (value16 > value16_1) {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if (value16_1 > value16_2) {
        value16_3 = value16_1;
        value16_1 = value16_2;
        value16_2 = value16_3;
    }

    if (value16 > value16_1) {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    voltage = value16_1 * 312 / 1024;

    return voltage;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity = cw_get_capacity(cw_bat);

    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
        cw_bat->bat_change = 1;
        cw_update_time_member_capacity_change(cw_bat);
    }
}

static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
    int ret = cw_get_vol(cw_bat);

    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
        cw_bat->bat_change = 1;
    }
}

static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work = container_of(work, struct delayed_work, work);
    struct cw_battery *cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

    rk_usb_update_online(cw_bat);
    rk_bat_update_capacity(cw_bat);
    rk_bat_update_vol(cw_bat);

    cw2015_capacity = cw_bat->capacity;
    cw2015_vol = cw_bat->voltage;

    if (cw_bat->bat_change)
        cw_bat->bat_change = 0;

    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));
}

static int cw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cw_battery *cw_bat;
    int ret, loop;

    mt_set_gpio_mode(GPIO_I2C4_SDA_PIN, GPIO_I2C4_SDA_PIN_M_SDA);
    mt_set_gpio_mode(GPIO_I2C4_SCA_PIN, GPIO_I2C4_SCA_PIN_M_SCL);

    cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
    if (!cw_bat)
        return -ENOMEM;

    i2c_set_clientdata(client, cw_bat);

    cw_bat->usb_online = 0;
    cw_bat->charger_mode = 0;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;
    cw_bat->bat_change = 0;

    cw_bat->plat_data = client->dev.platform_data;
    cw_bat->client = client;
    cw_bat->plat_data = &cw_bat_platdata;

    ret = cw_init(cw_bat);
    while ((loop++ < 2000) && (ret != 0))
        ret = cw_init(cw_bat);

    if (ret)
        return ret;

    cw_update_time_member_capacity_change(cw_bat);
    cw_update_time_member_charge_start(cw_bat);

    cw_bat->battery_workqueue = create_singlethread_workqueue("cw_battery");
    INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));

    return 0;
}

static int cw2015_i2c_remove(struct i2c_client *client)
{
    struct cw_battery *data = i2c_get_clientdata(client);
    cancel_delayed_work(&data->battery_delay_work);
    i2c_unregister_device(client);
    kfree(data);

    return 0;
}

static int cw2015_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, "cw2015");

    return 0;
}

static int cw2015_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    cancel_delayed_work(&cw_bat->battery_delay_work);

    return 0;
}

static int cw2015_i2c_resume(struct i2c_client *client)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));

    return 0;
}

static const struct i2c_device_id cw2015_i2c_id[] = {{"cw2015", 0}, {}};
static struct i2c_board_info __initdata i2c_cw2015 = {I2C_BOARD_INFO("cw2015", 0x62)};

static struct i2c_driver cw2015_i2c_driver = {
    .probe = cw2015_i2c_probe,
    .remove = cw2015_i2c_remove,
    .detect = cw2015_i2c_detect,
    .suspend = cw2015_i2c_suspend,
    .resume = cw2015_i2c_resume,
    .id_table = cw2015_i2c_id,
    .driver = {
        .name = "cw2015"
    }
};

static int __init cw_bat_init(void)
{
    i2c_register_board_info(4, &i2c_cw2015, 1);
    if (i2c_add_driver(&cw2015_i2c_driver)) {
        printk("[CW2015] Error adding driver to i2c\n");
        return -1;
    }

    return 0;
}

static void __exit cw_bat_exit(void)
{
    i2c_del_driver(&cw2015_i2c_driver);
}

module_init(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("<ben.chen@cellwise-semi.com>");
MODULE_DESCRIPTION("CW2015 Battery Driver");
MODULE_LICENSE("GPL");
