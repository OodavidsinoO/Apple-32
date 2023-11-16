#include "ff.h"

WCHAR ff_convert (WCHAR wch, UINT dir)
{
          if (wch < 0x80) {
                    return wch;
          }

          // we are not supporting unicode
          return 0;
}

WCHAR ff_wtoupper (WCHAR wch)
{
          if (wch < 0x80) {
                    if (wch >= 'a' && wch <= 'z') {
                              wch &= ~0x20;
                     }
                      return wch;
          }

          // we are not supporting unicode
          return 0;
}
