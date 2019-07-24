
#include "bsp_header.h"
#include "spi.h"
#include "bsp_norflash.h"

#define CS_High(GPIO, Pin) (((GPIO_TypeDef *)GPIO)->BSRR = Pin)
#define CS_Low(GPIO, Pin)  (((GPIO_TypeDef *)GPIO)->BSRR = Pin << 16)

#define NOR_SECTOR_SIZE (4 * 1024)

static char nor_buf[NOR_SECTOR_SIZE] = {0};

static const NORFLASH_DESC Descs[] = {
    {"W25Q128",   0xEF4018, 256, 4, 64, 16 * 1024},
    {"W25Q64",    0xEF4017, 256, 4, 64,  8 * 1024},
    {"W25Q32",    0xEF4016, 256, 4, 64,  4 * 1024},
    {"MX25L3206", 0xC22016, 256, 4, 64,  4 * 1024},
};

#define DESCS_NUM (sizeof(Descs)/sizeof(NORFLASH_DESC))

static void *find_desc(int jedec)
{
    void *desc = NULL;
    int i = 0;

    for (i = 0; i < DESCS_NUM; i++)
    {
        if (jedec == Descs[i].Jedec)
        {
            desc = (void *)&Descs[i];
            break;
        }
    }

    return desc;
}

static void wait_write_enable(void *obj)
{
    NORFLASH_OBJ *Obj = obj;
    uint8_t status = 0;

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x05}, 1, HAL_MAX_DELAY);
    do
        HAL_SPI_Receive(Obj->Handle, &status, 1, HAL_MAX_DELAY);
    while (!(status & 0x02));
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);
}

static void write_enable(void *obj)
{
    NORFLASH_OBJ *Obj = obj;
    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x06}, 1, HAL_MAX_DELAY);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);
}

static void wait_busy(void *obj)
{
    NORFLASH_OBJ *Obj = obj;
    uint8_t status = 0;

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x05}, 1, HAL_MAX_DELAY);
    do
        HAL_SPI_Receive(Obj->Handle, &status, 1, HAL_MAX_DELAY);
    while (status & 0x01);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);
}

static void data_read(void *obj, int addr, void *buf, int length)
{
    if (obj == NULL)
        return;

    NORFLASH_OBJ *Obj = obj;

    if (Obj->Desc == NULL)
        return;

    addr <<= 8;
    addr = __REV(addr);

    wait_busy(obj);

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x03}, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t *)&addr, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(Obj->Handle, buf, length, HAL_MAX_DELAY);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);
}

static int check_blank(void *obj, int addr, int length)
{
    data_read(obj, addr, nor_buf, length);

    for (int i = 0; i < length; i++)
        if (nor_buf[i] != 0xFF)
            return -1;

    return 0;
}

static void sector_erase(void *obj, int addr)
{
    NORFLASH_OBJ *Obj = obj;

    addr <<= 8;
    addr = __REV(addr);

    write_enable(obj);
    wait_write_enable(obj);

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x20}, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t *)&addr, 3, HAL_MAX_DELAY);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);

    wait_busy(obj);
}

static void page_program(void *obj, int addr, void *buf, int length)
{
    NORFLASH_OBJ *Obj = obj;

    addr <<= 8;
    addr = __REV(addr);

    write_enable(obj);
    wait_write_enable(obj);

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x02}, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t *)&addr, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(Obj->Handle, buf, length, HAL_MAX_DELAY);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);

    wait_busy(obj);
}

static void data_write(void *obj, int addr, void *buf, int length)
{
    if (obj == NULL)
        return;

    NORFLASH_OBJ *Obj = obj;

    if (Obj->Desc == NULL)
        return;

    int  Length = 0;
    int  inPageLength = 0;
    char *pData = NULL;
    char *wData = buf;

    while(length > 0)
    {
        if((addr % (Obj->Desc->SecSizeK * 1024)) + length > (Obj->Desc->SecSizeK * 1024))
            Length = (Obj->Desc->SecSizeK * 1024) - (addr % (Obj->Desc->SecSizeK * 1024));
        else
            Length = length;

        //判断写入位置是否为空
        if(check_blank(obj, addr, Length) == 0)
        {
            //为空时，直接写入到NorFlash
            while(Length > 0)
            {
                //写入数据截断在Page内
                if((addr % Obj->Desc->PgSizeB) + Length > Obj->Desc->PgSizeB)
                    inPageLength = Obj->Desc->PgSizeB - (addr % Obj->Desc->PgSizeB);
                else
                    inPageLength = Length;

                page_program(obj, addr, wData, inPageLength);

                addr   += inPageLength;
                wData  += inPageLength;
                Length -= inPageLength;
                length -= inPageLength;
            }
        }
        else
        {
            //为非空时，将数据拷贝至Buffer对应位置
            data_read(obj, addr & 0xFFFFF000, nor_buf, Obj->Desc->SecSizeK * 1024);

            pData = nor_buf + addr % (Obj->Desc->SecSizeK * 1024);

            for(uint32_t i = 0; i < Length; i++)
                *pData++ = *wData++;

            //指针指向Buffer
            pData = nor_buf;

            length -= Length;
            addr   &= 0xFFFFF000;
            Length  = Obj->Desc->SecSizeK * 1024;

            //擦除写入位置所在Sector
            sector_erase(obj, addr);

            //写入Buffer
            while(Length > 0)
            {
                page_program(obj, addr, pData, Obj->Desc->PgSizeB);

                addr   += Obj->Desc->PgSizeB;
                pData  += Obj->Desc->PgSizeB;
                Length -= Obj->Desc->PgSizeB;
            }
        }
    }
}

static void soft_reset(void *obj)
{
    NORFLASH_OBJ *Obj = obj;

    if (Obj->Desc == NULL)
        return;

    if (Obj->Desc->Jedec & 0xFF0000 == 0xEF0000)
    {
        CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
        HAL_SPI_Transmit(Obj->Handle, (uint8_t[]){0x66}, 1, HAL_MAX_DELAY);
        CS_High(Obj->CS.GPIO, Obj->CS.Pin);

        CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
        HAL_SPI_Transmit(Obj->Handle, (uint8_t[]){0x99}, 1, HAL_MAX_DELAY);
        CS_High(Obj->CS.GPIO, Obj->CS.Pin);
    }
}

static int read_jedec(void *obj)
{
    NORFLASH_OBJ *Obj = obj;
    int jedec = 0;

    CS_Low(Obj->CS.GPIO, Obj->CS.Pin);
    HAL_SPI_Transmit(Obj->Handle, (uint8_t []){0x9F}, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(Obj->Handle, (uint8_t *)&jedec, 4, HAL_MAX_DELAY);
    CS_High(Obj->CS.GPIO, Obj->CS.Pin);

    return __REV(jedec << 8);
}

static void init(void *obj)
{
    if (obj == NULL)
        return;

    NORFLASH_OBJ *Obj = obj;

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin   = Obj->CS.Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    CS_High(Obj->CS.GPIO, Obj->CS.Pin);

    HAL_GPIO_Init(Obj->CS.GPIO, &GPIO_InitStruct);

    Obj->Desc = find_desc(read_jedec(obj));

    soft_reset(obj);
}

void *BSP_NORFLASH_API(void)
{
    static NORFLASH_API api = {
        .Init      = init,
        .DataRead  = data_read,
        .DataWrite = data_write,
    };

    return &api;
}
