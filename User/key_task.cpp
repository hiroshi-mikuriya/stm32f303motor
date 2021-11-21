/// @file      User/key_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "gpio.hpp"
#include "main.h"

namespace
{
/// @brief キーレベル
struct KeyLevel
{
  bool m[COUNT_OF_MOTORS]; ///< レベル
};
/// @brief キーレベルを取得する @return キーレベル
KeyLevel getKeyLevel()
{
  KeyLevel level;
  level.m[0] = InputPin{BTN1_GPIO_Port, BTN1_Pin}.level();
  level.m[1] = InputPin{BTN2_GPIO_Port, BTN2_Pin}.level();
  return level;
};

KeyLevel s_preKey{}; ///< ボタン状態を前回値
} // namespace

extern "C"
{
  /// @brief KEYタスク初期化
  void keyTaskInit(void)
  {
    LL_TIM_EnableCounter(KEY_CHECK_TIM);
    LL_TIM_EnableIT_UPDATE(KEY_CHECK_TIM);
  }
  /// @brief KEYタスク<br>
  /// キー入力を監視し、変化があればメッセージを送信する
  /// @param [in] res タスク共有リソース
  void keyTaskProc(TaskResource *res)
  {
    if (LL_TIM_IsActiveFlag_UPDATE(KEY_CHECK_TIM))
    {
      LL_TIM_ClearFlag_UPDATE(KEY_CHECK_TIM);
      auto cur = getKeyLevel();
      for (uint32_t i = 0; i < COUNT_OF_MOTORS; ++i)
      {
        if (s_preKey.m[i] && !cur.m[i])
        {
          res[i].btnPushedEvent = 1;
        }
      }
      s_preKey = cur;
    }
  }
}
