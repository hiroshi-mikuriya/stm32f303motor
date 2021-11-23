/// @file      User/motor_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "gpio.hpp"
#include "main.h"

namespace
{
/// @brief モータータスククラス
class MotorTask
{
  InputPin swt_;                                     ///< モーター回転方向スイッチ
  OutputPin led_;                                    ///< LED
  OutputPin ctrl1_;                                  ///< モータドライバ制御1
  OutputPin ctrl2_;                                  ///< モータドライバ制御2
  TIM_TypeDef *pwmTim_;                              ///< PWM制御ペリフェラル
  void (*setCompareValue_)(TIM_TypeDef *, uint32_t); ///< PWM値設定関数
  bool running_;

  /// @brief GPIO出力を更新する
  void updateGpio()
  {
    if (!running_)
    {
      ctrl1_.low();
      ctrl2_.low();
      led_.low();
    }
    else if (swt_.level())
    {
      ctrl1_.high();
      ctrl2_.low();
      led_.high();
    }
    else
    {
      ctrl1_.low();
      ctrl2_.high();
      led_.high();
    }
  }

public:
  /// @brief デフォルトコンストラクタ
  MotorTask() : swt_({}), led_({}), ctrl1_({}), ctrl2_({}), pwmTim_(0), setCompareValue_(0), running_(false) {}
  /// @brief プロパティ初期化
  /// @param [in] swt モーター回転方向スイッチ
  /// @param [in] led LED
  /// @param [in] ctrl1 モータドライバ制御1
  /// @param [in] ctrl2 モータドライバ制御2
  /// @param [in] pwmTim PWM制御ペリフェラル
  /// @param [in] setCompareValue PWM値設定関数
  void setProperty(                                    //
      InputPin const &swt,                             //
      OutputPin const &led,                            //
      OutputPin const &ctrl1,                          //
      OutputPin const &ctrl2,                          //
      TIM_TypeDef *pwmTim,                             //
      void (*setCompareValue)(TIM_TypeDef *, uint32_t) //
  )
  {
    swt_ = swt;
    led_ = led;
    ctrl1_ = ctrl1;
    ctrl2_ = ctrl2;
    pwmTim_ = pwmTim;
    setCompareValue_ = setCompareValue;
  }

  /// @brief タスク処理実行 @param [in] res タスク共通リソース
  void run(TaskResource const &res)
  {
    if (res.btnPushedEvent)
    {
      running_ = !running_;
    }
    updateGpio();
    setCompareValue_(pwmTim_, res.analog);
  }
};
/// モータータスククラスインスタンス
MotorTask s_motorTask[COUNT_OF_MOTORS];
} // namespace

extern "C"
{
  /// @brief モータタスク初期化
  void motorTaskInit(void)
  {
    s_motorTask[0].setProperty(                              //
        InputPin{SWT1_GPIO_Port, SWT1_Pin},                  //
        OutputPin{LED1_GPIO_Port, LED1_Pin},                 //
        OutputPin{MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin}, //
        OutputPin{MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin}, //
        MOTOR_PWM_TIM,                                       //
        LL_TIM_OC_SetCompareCH1                              //
    );
    s_motorTask[1].setProperty(                              //
        InputPin{SWT2_GPIO_Port, SWT2_Pin},                  //
        OutputPin{LED2_GPIO_Port, LED2_Pin},                 //
        OutputPin{MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin}, //
        OutputPin{MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin}, //
        MOTOR_PWM_TIM,                                       //
        LL_TIM_OC_SetCompareCH2                              //
    );
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(MOTOR_PWM_TIM, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(MOTOR_PWM_TIM);
    LL_TIM_EnableAllOutputs(MOTOR_PWM_TIM);
  }
  /// @brief モータタスク
  /// @param [in] res タスク共有リソース
  void motorTaskProc(TaskResource *res)
  {
    for (int i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      s_motorTask[i].run(res[i]);
    }
  }
}
