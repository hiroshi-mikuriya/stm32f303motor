/// @file      User/encoder_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "main.h"

namespace
{
/// エンコーダ読み取りクラス
class EncoderInfo
{
  TIM_TypeDef *tim_;   ///< TIM
  int32_t preCounter_; ///< 前回値
  int64_t counter_;    ///< エンコーダ累積値

public:
  /// @brief デフォルトコンストラクタ
  EncoderInfo() : tim_(0), preCounter_(0), counter_(0) {}
  /// @brief TIMを初期化する @param [in] tim TIMペリフェラル
  void initTim(TIM_TypeDef *tim)
  {
    tim_ = tim;
    LL_TIM_SetCounter(tim_, 0);
    LL_TIM_EnableCounter(tim_);
  }
  /// @brief エンコーダ値を更新する
  /// @note TIMのカウンターが２回転する前にこの関数を呼び出さないとエンコーダ情報を取りこぼす。
  void update()
  {
    int32_t cnt = static_cast<int32_t>(LL_TIM_GetCounter(tim_));
    counter_ += cnt - preCounter_;
    preCounter_ = cnt;
  }
  /// @brief エンコーダ値を取得する @return エンコーダ値
  int64_t get() const { return counter_; }
};
// エンコーダ読み取りクラスインスタンス
EncoderInfo s_encoders[COUNT_OF_MOTORS];
} // namespace

extern "C"
{
  /// @brief エンコーダタスク初期化
  void encoderTaskInit(void)
  {
    s_encoders[0].initTim(MOTOR1_ENC_TIM);
    s_encoders[1].initTim(MOTOR2_ENC_TIM);
    LL_TIM_EnableCounter(ENC_UPDATE_TIM);
    LL_TIM_EnableIT_UPDATE(ENC_UPDATE_TIM);
  }
  /// @brief エンコーダタスク<br>
  /// エンコーダ値を定期的に読み取りメッセージを送信する
  /// @param [in] res タスク共有リソース
  void encoderTaskProc(TaskResource *res)
  {
    constexpr IRQn_Type IRQn = TIM6_DAC1_IRQn;
    NVIC_DisableIRQ(IRQn);
    for (int i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      res[i].encoder = s_encoders[i].get();
    }
    NVIC_EnableIRQ(IRQn);
  }
  /// @brief エンコーダ値読み出し通知割り込み
  void encoderUpdateTimIRQ(void)
  {
    for (auto &e : s_encoders)
    {
      e.update();
    }
  }
}
