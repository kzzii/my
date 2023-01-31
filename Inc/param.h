#ifndef __PARAM_H
#define __PARAM_H

/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
/*----------------------------------------------------------------------------*/


#define BANK1_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Page 0 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Page 1 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Page 2 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Page 3 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Page 4 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Page 5 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Page 6 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Page 7 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Page 8 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Page 9 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_10    ((uint32_t)0x08005000) /* Page 10 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_11    ((uint32_t)0x08005800) /* Page 11 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_12    ((uint32_t)0x08006000) /* Page 12 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_13    ((uint32_t)0x08006800) /* Page 13 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_14    ((uint32_t)0x08007000) /* Page 14 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_15    ((uint32_t)0x08007800) /* Page 15 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_16    ((uint32_t)0x08008000) /* Page 16 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_17    ((uint32_t)0x08008800) /* Page 17 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_18    ((uint32_t)0x08009000) /* Page 18 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_19    ((uint32_t)0x08009800) /* Page 19 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_20    ((uint32_t)0x0800A000) /* Page 20 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_21    ((uint32_t)0x0800A800) /* Page 21 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_22    ((uint32_t)0x0800B000) /* Page 22 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_23    ((uint32_t)0x0800B800) /* Page 23 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_24    ((uint32_t)0x0800C000) /* Page 24 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_25    ((uint32_t)0x0800C800) /* Page 25 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_26    ((uint32_t)0x0800D000) /* Page 26 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_27    ((uint32_t)0x0800D800) /* Page 27 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_28    ((uint32_t)0x0800E000) /* Page 28 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_29    ((uint32_t)0x0800E800) /* Page 29 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_30    ((uint32_t)0x0800F000) /* Page 30 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_31    ((uint32_t)0x0800F800) /* Page 31 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_32    ((uint32_t)0x08010000) /* Page 32 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_33    ((uint32_t)0x08010800) /* Page 33 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_34    ((uint32_t)0x08011000) /* Page 34 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_35    ((uint32_t)0x08011800) /* Page 35 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_36    ((uint32_t)0x08012000) /* Page 36 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_37    ((uint32_t)0x08012800) /* Page 37 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_38    ((uint32_t)0x08013000) /* Page 38 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_39    ((uint32_t)0x08013800) /* Page 39 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_40    ((uint32_t)0x08014000) /* Page 40 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_41    ((uint32_t)0x08014800) /* Page 41 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_42    ((uint32_t)0x08015000) /* Page 42 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_43    ((uint32_t)0x08015800) /* Page 43 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_44    ((uint32_t)0x08016000) /* Page 44 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_45    ((uint32_t)0x08016800) /* Page 45 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_46    ((uint32_t)0x08017000) /* Page 46 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_47    ((uint32_t)0x08017800) /* Page 47 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_48    ((uint32_t)0x08018000) /* Page 48 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_49    ((uint32_t)0x08018800) /* Page 49 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_50    ((uint32_t)0x08019000) /* Page 50 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_51    ((uint32_t)0x08019800) /* Page 51 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_52    ((uint32_t)0x0801A000) /* Page 52 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_53    ((uint32_t)0x0801A800) /* Page 53 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_54    ((uint32_t)0x0801B000) /* Page 54 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_55    ((uint32_t)0x0801B800) /* Page 55 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_56    ((uint32_t)0x0801C000) /* Page 56 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_57    ((uint32_t)0x0801C800) /* Page 57 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_58    ((uint32_t)0x0801D000) /* Page 58 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_59    ((uint32_t)0x0801D800) /* Page 59 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_60    ((uint32_t)0x0801E000) /* Page 60 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_61    ((uint32_t)0x0801E800) /* Page 61 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_62    ((uint32_t)0x0801F000) /* Page 62 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_63    ((uint32_t)0x0801F800) /* Page 63 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_64    ((uint32_t)0x08020000) /* Page 64 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_65    ((uint32_t)0x08020800) /* Page 65 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_66    ((uint32_t)0x08021000) /* Page 66 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_67    ((uint32_t)0x08021800) /* Page 67 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_68    ((uint32_t)0x08022000) /* Page 68 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_69    ((uint32_t)0x08022800) /* Page 69 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_70    ((uint32_t)0x08023000) /* Page 70 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_71    ((uint32_t)0x08023800) /* Page 71 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_72    ((uint32_t)0x08024000) /* Page 72 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_73    ((uint32_t)0x08024800) /* Page 73 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_74    ((uint32_t)0x08025000) /* Page 74 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_75    ((uint32_t)0x08025800) /* Page 75 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_76    ((uint32_t)0x08026000) /* Page 76 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_77    ((uint32_t)0x08026800) /* Page 77 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_78    ((uint32_t)0x08027000) /* Page 78 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_79    ((uint32_t)0x08027800) /* Page 79 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_80    ((uint32_t)0x08028000) /* Page 80 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_81    ((uint32_t)0x08028800) /* Page 81 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_82    ((uint32_t)0x08029000) /* Page 82 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_83    ((uint32_t)0x08029800) /* Page 83 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_84    ((uint32_t)0x0802A000) /* Page 84 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_85    ((uint32_t)0x0802A800) /* Page 85 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_86    ((uint32_t)0x0802B000) /* Page 86 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_87    ((uint32_t)0x0802B800) /* Page 87 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_88    ((uint32_t)0x0802C000) /* Page 88 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_89    ((uint32_t)0x0802C800) /* Page 89 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_90    ((uint32_t)0x0802D000) /* Page 90 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_91    ((uint32_t)0x0802D800) /* Page 91 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_92    ((uint32_t)0x0802E000) /* Page 92 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_93    ((uint32_t)0x0802E800) /* Page 93 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_94    ((uint32_t)0x0802F000) /* Page 94 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_95    ((uint32_t)0x0802F800) /* Page 95 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_96    ((uint32_t)0x08030000) /* Page 96 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_97    ((uint32_t)0x08030800) /* Page 97 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_98    ((uint32_t)0x08031000) /* Page 98 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_99    ((uint32_t)0x08031800) /* Page 99 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_100   ((uint32_t)0x08032000) /* Page 100 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_101   ((uint32_t)0x08032800) /* Page 101 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_102   ((uint32_t)0x08033000) /* Page 102 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_103   ((uint32_t)0x08033800) /* Page 103 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_104   ((uint32_t)0x08034000) /* Page 104 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_105   ((uint32_t)0x08034800) /* Page 105 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_106   ((uint32_t)0x08035000) /* Page 106 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_107   ((uint32_t)0x08035800) /* Page 107 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_108   ((uint32_t)0x08036000) /* Page 108 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_109   ((uint32_t)0x08036800) /* Page 109 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_110   ((uint32_t)0x08037000) /* Page 110 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_111   ((uint32_t)0x08037800) /* Page 111 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_112   ((uint32_t)0x08038000) /* Page 112 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_113   ((uint32_t)0x08038800) /* Page 113 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_114   ((uint32_t)0x08039000) /* Page 114 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_115   ((uint32_t)0x08039800) /* Page 115 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_116   ((uint32_t)0x0803A000) /* Page 116 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_117   ((uint32_t)0x0803A800) /* Page 117 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_118   ((uint32_t)0x0803B000) /* Page 118 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_119   ((uint32_t)0x0803B800) /* Page 119 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_120   ((uint32_t)0x0803C000) /* Page 120 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_121   ((uint32_t)0x0803C800) /* Page 121 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_122   ((uint32_t)0x0803D000) /* Page 122 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_123   ((uint32_t)0x0803D800) /* Page 123 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_124   ((uint32_t)0x0803E000) /* Page 124 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_125   ((uint32_t)0x0803E800) /* Page 125 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_126   ((uint32_t)0x0803F000) /* Page 126 of BANK1, 2 Kbytes */
#define BANK1_FLASH_PAGE_127   ((uint32_t)0x0803F800) /* Page 127 of BANK1, 2 Kbytes */

#define BANK2_FLASH_PAGE_0     ((uint32_t)0x08040000) /*  Page 0 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_1     ((uint32_t)0x08040800) /*  Page 1 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_2     ((uint32_t)0x08041000) /*  Page 2 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_3     ((uint32_t)0x08041800) /*  Page 3 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_4     ((uint32_t)0x08042000) /*  Page 4 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_5     ((uint32_t)0x08042800) /*  Page 5 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_6     ((uint32_t)0x08043000) /*  Page 6 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_7     ((uint32_t)0x08043800) /*  Page 7 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_8     ((uint32_t)0x08044000) /*  Page 8 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_9     ((uint32_t)0x08044800) /*  Page 9 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_10    ((uint32_t)0x08045000) /*  Page 10 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_11    ((uint32_t)0x08045800) /*  Page 11 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_12    ((uint32_t)0x08046000) /*  Page 12 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_13    ((uint32_t)0x08046800) /*  Page 13 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_14    ((uint32_t)0x08047000) /*  Page 14 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_15    ((uint32_t)0x08047800) /*  Page 15 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_16    ((uint32_t)0x08048000) /*  Page 16 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_17    ((uint32_t)0x08048800) /*  Page 17 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_18    ((uint32_t)0x08049000) /*  Page 18 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_19    ((uint32_t)0x08049800) /*  Page 19 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_20    ((uint32_t)0x0804a000) /*  Page 20 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_21    ((uint32_t)0x0804a800) /*  Page 21 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_22    ((uint32_t)0x0804b000) /*  Page 22 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_23    ((uint32_t)0x0804b800) /*  Page 23 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_24    ((uint32_t)0x0804c000) /*  Page 24 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_25    ((uint32_t)0x0804c800) /*  Page 25 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_26    ((uint32_t)0x0804d000) /*  Page 26 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_27    ((uint32_t)0x0804d800) /*  Page 27 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_28    ((uint32_t)0x0804e000) /*  Page 28 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_29    ((uint32_t)0x0804e800) /*  Page 29 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_30    ((uint32_t)0x0804f000) /*  Page 30 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_31    ((uint32_t)0x0804f800) /*  Page 31 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_32    ((uint32_t)0x08050000) /*  Page 32 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_33    ((uint32_t)0x08050800) /*  Page 33 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_34    ((uint32_t)0x08051000) /*  Page 34 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_35    ((uint32_t)0x08051800) /*  Page 35 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_36    ((uint32_t)0x08052000) /*  Page 36 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_37    ((uint32_t)0x08052800) /*  Page 37 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_38    ((uint32_t)0x08053000) /*  Page 38 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_39    ((uint32_t)0x08053800) /*  Page 39 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_40    ((uint32_t)0x08054000) /*  Page 40 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_41    ((uint32_t)0x08054800) /*  Page 41 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_42    ((uint32_t)0x08055000) /*  Page 42 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_43    ((uint32_t)0x08055800) /*  Page 43 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_44    ((uint32_t)0x08056000) /*  Page 44 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_45    ((uint32_t)0x08056800) /*  Page 45 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_46    ((uint32_t)0x08057000) /*  Page 46 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_47    ((uint32_t)0x08057800) /*  Page 47 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_48    ((uint32_t)0x08058000) /*  Page 48 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_49    ((uint32_t)0x08058800) /*  Page 49 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_50    ((uint32_t)0x08059000) /*  Page 50 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_51    ((uint32_t)0x08059800) /*  Page 51 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_52    ((uint32_t)0x0805a000) /*  Page 52 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_53    ((uint32_t)0x0805a800) /*  Page 53 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_54    ((uint32_t)0x0805b000) /*  Page 54 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_55    ((uint32_t)0x0805b800) /*  Page 55 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_56    ((uint32_t)0x0805c000) /*  Page 56 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_57    ((uint32_t)0x0805c800) /*  Page 57 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_58    ((uint32_t)0x0805d000) /*  Page 58 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_59    ((uint32_t)0x0805d800) /*  Page 59 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_60    ((uint32_t)0x0805e000) /*  Page 60 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_61    ((uint32_t)0x0805e800) /*  Page 61 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_62    ((uint32_t)0x0805f000) /*  Page 62 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_63    ((uint32_t)0x0805f800) /*  Page 63 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_64    ((uint32_t)0x08060000) /*  Page 64 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_65    ((uint32_t)0x08060800) /*  Page 65 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_66    ((uint32_t)0x08061000) /*  Page 66 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_67    ((uint32_t)0x08061800) /*  Page 67 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_68    ((uint32_t)0x08062000) /*  Page 68 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_69    ((uint32_t)0x08062800) /*  Page 69 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_70    ((uint32_t)0x08063000) /*  Page 70 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_71    ((uint32_t)0x08063800) /*  Page 71 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_72    ((uint32_t)0x08064000) /*  Page 72 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_73    ((uint32_t)0x08064800) /*  Page 73 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_74    ((uint32_t)0x08065000) /*  Page 74 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_75    ((uint32_t)0x08065800) /*  Page 75 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_76    ((uint32_t)0x08066000) /*  Page 76 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_77    ((uint32_t)0x08066800) /*  Page 77 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_78    ((uint32_t)0x08067000) /*  Page 78 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_79    ((uint32_t)0x08067800) /*  Page 79 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_80    ((uint32_t)0x08068000) /*  Page 80 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_81    ((uint32_t)0x08068800) /*  Page 81 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_82    ((uint32_t)0x08069000) /*  Page 82 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_83    ((uint32_t)0x08069800) /*  Page 83 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_84    ((uint32_t)0x0806a000) /*  Page 84 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_85    ((uint32_t)0x0806a800) /*  Page 85 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_86    ((uint32_t)0x0806b000) /*  Page 86 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_87    ((uint32_t)0x0806b800) /*  Page 87 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_88    ((uint32_t)0x0806c000) /*  Page 88 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_89    ((uint32_t)0x0806c800) /*  Page 89 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_90    ((uint32_t)0x0806d000) /*  Page 90 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_91    ((uint32_t)0x0806d800) /*  Page 91 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_92    ((uint32_t)0x0806e000) /*  Page 92 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_93    ((uint32_t)0x0806e800) /*  Page 93 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_94    ((uint32_t)0x0806f000) /*  Page 94 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_95    ((uint32_t)0x0806f800) /*  Page 95 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_96    ((uint32_t)0x08070000) /*  Page 96 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_97    ((uint32_t)0x08070800) /*  Page 97 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_98    ((uint32_t)0x08071000) /*  Page 98 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_99    ((uint32_t)0x08071800) /*  Page 99 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_100   ((uint32_t)0x08072000) /*  Page 100 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_101   ((uint32_t)0x08072800) /*  Page 101 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_102   ((uint32_t)0x08073000) /*  Page 102 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_103   ((uint32_t)0x08073800) /*  Page 103 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_104   ((uint32_t)0x08074000) /*  Page 104 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_105   ((uint32_t)0x08074800) /*  Page 105 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_106   ((uint32_t)0x08075000) /*  Page 106 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_107   ((uint32_t)0x08075800) /*  Page 107 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_108   ((uint32_t)0x08076000) /*  Page 108 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_109   ((uint32_t)0x08076800) /*  Page 109 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_110   ((uint32_t)0x08077000) /*  Page 110 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_111   ((uint32_t)0x08077800) /*  Page 111 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_112   ((uint32_t)0x08078000) /*  Page 112 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_113   ((uint32_t)0x08078800) /*  Page 113 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_114   ((uint32_t)0x08079000) /*  Page 114 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_115   ((uint32_t)0x08079800) /*  Page 115 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_116   ((uint32_t)0x0807a000) /*  Page 116 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_117   ((uint32_t)0x0807a800) /*  Page 117 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_118   ((uint32_t)0x0807b000) /*  Page 118 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_119   ((uint32_t)0x0807b800) /*  Page 119 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_120   ((uint32_t)0x0807c000) /*  Page 120 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_121   ((uint32_t)0x0807c800) /*  Page 121 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_122   ((uint32_t)0x0807d000) /*  Page 122 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_123   ((uint32_t)0x0807d800) /*  Page 123 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_124   ((uint32_t)0x0807e000) /*  Page 124 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_125   ((uint32_t)0x0807e800) /*  Page 125 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_126   ((uint32_t)0x0807f000) /*  Page 126 of BANK2, 2 Kbytes */
#define BANK2_FLASH_PAGE_127   ((uint32_t)0x0807f800) /*  Page 127 of BANK2, 2 Kbytes */



typedef enum {
    PARAM_INDEX_CELL1_CELLOUT0,
    PARAM_INDEX_CELL1_CELLOUT1,
    PARAM_INDEX_CELL1_CURRENT0,
    PARAM_INDEX_CELL1_CURRENT1,
    PARAM_INDEX_CELL1_CURRENT2,
    PARAM_INDEX_CELL1_VOLTAGE0,
    PARAM_INDEX_CELL1_VOLTAGE1,

    PARAM_INDEX_CELL2_CELLOUT0,
    PARAM_INDEX_CELL2_CELLOUT1,
    PARAM_INDEX_CELL2_CURRENT0,
    PARAM_INDEX_CELL2_CURRENT1,
    PARAM_INDEX_CELL2_CURRENT2,
    PARAM_INDEX_CELL2_VOLTAGE0,
    PARAM_INDEX_CELL2_VOLTAGE1,

    PARAM_INDEX_SERIAL0,
    PARAM_INDEX_SERIAL1,

    PARAM_INDEX_LEN,
} param_index_t;

extern void calData_setValue(uint16_t index, uint16_t value);
extern uint16_t calData_value(uint16_t index);

/*----------------------------------------------------------------------------*/

void param_init(void);

bool param_exist(void);

uint32_t param_get_param(const size_t index);
void param_set_param(const size_t index, uint32_t value);

float param_get_param_float(const size_t index);

extern void calData_setValue(uint16_t index, uint16_t value);
extern uint16_t calData_value(uint16_t index);

extern void param_load(void);
extern void param_save(void);
extern void param_erase(void);

/*----------------------------------------------------------------------------*/

#endif // __PARAM_H
