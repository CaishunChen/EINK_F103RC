
#ifndef _BSP_NORFLASH_H_
#define _BSP_NORFLASH_H_

typedef struct _NORFLASH_DESC {
    char *Name;
    int   Jedec;
    int   PgSizeB;
    int   SecSizeK;
    int   BlkSizeK;
    int   SizeK;
} NORFLASH_DESC;

typedef struct _NORFLASH_CS {
    void *GPIO;
    int   Pin;
} NORFLASH_CS;

typedef struct _NORFLASH_OBJ {
    void          *Handle;
    NORFLASH_CS    CS;
    NORFLASH_DESC *Desc;
} NORFLASH_OBJ;

typedef struct _NORFLASH_API {
    void (*Init)(void *);
    void (*DataRead)(void *, int, void *, int);
    void (*DataWrite)(void *, int, void *, int);
    void (*BlockErase)(void *, int);
    void (*ChipErase)(void *);
} NORFLASH_API;

void *BSP_NORFLASH_API(void);

#endif
