#include "bsp_flash.h"
#include "n32g430.h"

#define FLASH_SIZE 0x10000

// 获取指定地址所在页的大小
static uint32_t get_page_size(uint32_t address) {
    return 2 * 1024;
}

/**
 * @brief 擦除指定地址的Flash页
 * @param address: 要擦除的Flash地址
 * @return 0: 成功，其他值: 错误码
 */
static uint8_t fmc_erase_page(uint32_t address) {
    /* 检查地址是否在Flash范围内 */
    if (address < FLASH_BASE || address >= (FLASH_BASE + FLASH_SIZE)) {
        return 1;  // 地址超出范围
    }
    
    /* 解锁Flash控制器 */
    FLASH_Unlock();
    
    /* 清除所有标志位 */
    FLASH_Flag_Status_Clear(FLASH_FLAG_EOP);
    FLASH_Flag_Status_Clear(FLASH_FLAG_WRPERR);
    FLASH_Flag_Status_Clear(FLASH_FLAG_PGERR);
    
    /* 擦除页 */
    if (FLASH_EOP != FLASH_One_Page_Erase(address)){
        FLASH_Lock();
        return 2;  // 擦除失败
    }
    
    /* 等待擦除完成 */
    while (FLASH_Flag_Status_Get(FLASH_FLAG_BUSY));
    
    /* 锁定Flash控制器 */
    FLASH_Lock();
    return 0;  // 成功
}

/**
 * @brief 向Flash写入数据（支持跨页写入）
 * @param address: 目标Flash地址（需按字对齐）
 * @param data: 源数据指针
 * @param length: 数据长度（字节）
 * @return 0: 成功，其他值: 错误码
 */
uint8_t fmc_write(uint32_t address, uint8_t* data, uint32_t length) 
{
    uint32_t remaining = length;
    uint32_t offset = 0;
    uint32_t current_addr = address;
    
    /* 检查地址是否在Flash范围内且对齐 */
    if (address < FLASH_BASE || address >= (FLASH_BASE + FLASH_SIZE) || 
        (address % 4) != 0) {
        return 1;  // 地址无效
    }
    
    /* 解锁Flash控制器 */
    FLASH_Unlock();
    
    /* 清除所有标志位 */
    FLASH_Flag_Status_Clear(FLASH_FLAG_EOP);
    FLASH_Flag_Status_Clear(FLASH_FLAG_WRPERR);
    FLASH_Flag_Status_Clear(FLASH_FLAG_PGERR);
    
    /* 循环处理每个页 */
    while (remaining > 0) {
        uint32_t page_size = get_page_size(current_addr);
        uint32_t page_addr = current_addr & (~(page_size - 1));  // 页对齐地址
        uint32_t bytes_in_page = page_addr + page_size - current_addr;  // 当前页剩余空间
        
        /* 计算当前页可写入的最大字节数 */
        uint32_t bytes_to_write = (remaining < bytes_in_page) ? remaining : bytes_in_page;
        
        /* 确保写入长度是4的倍数（按字对齐） */
        bytes_to_write = (bytes_to_write / 4) * 4;
        if (bytes_to_write == 0) break;
        
        /* 擦除当前页 */
        if (fmc_erase_page(page_addr) != 0) {
            FLASH_Lock();
            return 2;  // 擦除失败
        }
        
        FLASH_Unlock();
        /* 按字（32位）写入数据 */
        for (uint32_t i = 0; i < bytes_to_write; i += 4) {
            uint32_t word_data = 0;
            
            /* 组装32位数据 */
            if (i + 3 < bytes_to_write) {
                word_data = (data[offset + i] << 0) | (data[offset + i + 1] << 8) | 
                           (data[offset + i + 2] << 16) | (data[offset + i + 3] << 24);
            } else if (i + 2 < bytes_to_write) {
                word_data = (data[offset + i] << 0) | (data[offset + i + 1] << 8) | 
                           (data[offset + i + 2] << 16);
            } else if (i + 1 < bytes_to_write) {
                word_data = (data[offset + i] << 0) | (data[offset + i + 1] << 8);
            } else {
                word_data = data[offset + i];
            }
            
            /* 写入字 */
            if (FLASH_EOP != FLASH_Word_Program(current_addr + i, word_data)){
                FLASH_Lock();
                return 3;  // 写入失败
            }
            
            /* 等待写入完成 */
            while (FLASH_Flag_Status_Get(FLASH_FLAG_BUSY));
        }
        
        /* 更新剩余字节数和偏移量 */
        remaining -= bytes_to_write;
        offset += bytes_to_write;
        current_addr += bytes_to_write;
    }
    
    /* 锁定Flash控制器 */
    FLASH_Lock();
    return 0;  // 成功
}

/**
 * @brief 从Flash读取数据
 * @param address: 源Flash地址
 * @param data: 目标缓冲区指针
 * @param length: 数据长度（字节）
 * @return 0: 成功，其他值: 错误码
 */
uint8_t fmc_read(uint32_t address, uint8_t* data, uint32_t length) 
{
    uint32_t i;
    
    /* 检查地址是否在Flash范围内 */
    if (address < FLASH_BASE || address >= (FLASH_BASE + FLASH_SIZE) || 
        (address + length) > (FLASH_BASE + FLASH_SIZE)) {
        return 1;  // 地址超出范围
    }
    
    /* 直接从Flash读取数据 */
    for (i = 0; i < length; i++) {
        data[i] = *((uint8_t*)(address + i));
    }
    
    return 0;  // 成功
}








