#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <mapmem.h>

int pin_mux(void)
{
    ulong addr;
    u32 value;
    const void *buf;

    addr = 0x05027048;
    buf = map_sysmem(addr, 4);
    writel(0x2320, (void *)buf);
    unmap_sysmem(buf);

    addr = 0x0502703c;
    buf = map_sysmem(addr, 4);
    writel(0x2320, (void *)buf);
    unmap_sysmem(buf);

    addr = 0x05027044;
    buf = map_sysmem(addr, 4);
    writel(0x2320, (void *)buf);
    unmap_sysmem(buf);

    addr = 0x05027040;
    buf = map_sysmem(addr, 4);
    writel(0x2320, (void *)buf);
    unmap_sysmem(buf);

    addr = 0x05021004;
    buf = map_sysmem(addr, 4);
    value = readl((void *)buf);
    value = value | 0x78;
    writel(value, (void *)buf);
    unmap_sysmem(buf);
    return 0;
}

int do_ledset(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    ulong addr;
    u32 value;
    const void *buf;

    if (argc != 3)
        return CMD_RET_USAGE;

    if ((!strncmp(argv[1], "status", 6)) && (!strncmp(argv[2], "on", 2))){
        pin_mux();
        addr = 0x05021000;
        buf = map_sysmem(addr, 4);
        value = readl((void *)buf);
        value = value | 0x10;
        value = value &~ 0x20;
        writel(value, (void *)buf);
        unmap_sysmem(buf);
    }
    else if ((!strncmp(argv[1], "status", 6)) && (!strncmp(argv[2], "off", 3))){
        pin_mux();
        addr = 0x05021000;
        buf = map_sysmem(addr, 4);
        value = readl((void *)buf);
        value = value &~ 0x10;
        writel(value, (void *)buf);
        unmap_sysmem(buf);
    }
    else if ((!strncmp(argv[1], "error", 5)) && (!strncmp(argv[2], "on", 2))){
        pin_mux();
        addr = 0x05021000;
        buf = map_sysmem(addr, 4);
        value = readl((void *)buf);
        value = value &~ 0x10;
        value = value | 0x20;
        writel(value, (void *)buf);
        unmap_sysmem(buf);
    }
    else if ((!strncmp(argv[1], "error", 5)) && (!strncmp(argv[2], "off", 2))){
        pin_mux();
        addr = 0x05021000;
        buf = map_sysmem(addr, 4);
        value = readl((void *)buf);
        value = value &~ 0x20;
        writel(value, (void *)buf);
        unmap_sysmem(buf);
    }
    else{
        return CMD_RET_USAGE;
    }
    return 0;
}

U_BOOT_CMD(
	led, 3, 0, do_ledset,
	"set SE9 leds on or off",
	"<led name> <stat>\n"
	"led status on\n"
	"led status off\n"
	"led error on\n"
	"led error off\n"
);
