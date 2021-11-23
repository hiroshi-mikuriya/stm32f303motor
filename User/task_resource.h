/// @file      User/task_resource.h
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#pragma once

#include <stdint.h>

/// モーター数定義
#define COUNT_OF_MOTORS 2

/// タスク共有リソース
typedef struct
{
  uint16_t btnPushedEvent; ///< ボタン押下イベント有無 @arg 0 イベントなし @arg 1 イベントあり
  uint16_t analog;         ///< アナログ値（0 - 1023）
  int64_t encoder;         ///< エンコーダ値
} TaskResource;
