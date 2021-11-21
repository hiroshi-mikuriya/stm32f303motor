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
typedef struct TaskResource
{
  uint16_t btnPushedEvent;
  uint16_t analog;
  int64_t encoder;
} TaskResource;
