/// @file      User/adc_task.cpp
/// @author    Hiroshi Mikuriya
/// @copyright Copyright© 2021 Hiroshi Mikuriya
///
/// DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.

#include "gpio.hpp"
#include "main.h"

namespace
{
/// DMAが書き込むバッファ
uint16_t s_dmaBuffer[COUNT_OF_MOTORS] = {0};
/// アナログ値
uint16_t s_analog[COUNT_OF_MOTORS] = {0};
} // namespace

extern "C"
{
  void adcTaskInit(void)
  {
    ADC_TypeDef *const adc = ADC2;
    DMA_TypeDef *const dma = DMA1;
    constexpr uint32_t channel = LL_DMA_CHANNEL_2;
    LL_DMA_ConfigAddresses(                                      //
        dma, channel,                                            //
        LL_ADC_DMA_GetRegAddr(adc, LL_ADC_DMA_REG_REGULAR_DATA), //
        reinterpret_cast<uint32_t>(s_dmaBuffer),                 //
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY                        //
    );
    LL_DMA_SetDataLength(dma, channel, COUNT_OF_MOTORS);
    LL_DMA_EnableIT_TC(dma, channel);
    LL_DMA_EnableIT_TE(dma, channel);
    LL_DMA_EnableChannel(dma, channel);
    LL_ADC_Enable(adc);
    LL_ADC_REG_StartConversion(adc);
  }
  /// @brief ADCタスク<br>
  /// 可変抵抗の抵抗値を定期的に読み取りメッセージを送信する
  /// @param [in] res タスク共有リソース
  void adcTaskProc(TaskResource *res)
  {
    constexpr IRQn_Type IRQn = DMA1_Channel2_IRQn;
    NVIC_DisableIRQ(IRQn);
    for (int i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      res[i].analog = s_analog[i];
    }
    NVIC_EnableIRQ(IRQn);
  }
  /// @brief ADC変換完了DMA割り込み
  void adcCpltIRQ(void)
  {
    for (uint32_t i = 0; i < COUNT_OF_MOTORS; ++i)
    {
      s_analog[i] = s_dmaBuffer[i];
    }
  }
  /// @brief ADC変換エラーDMA割り込み
  void adcErrorIRQ(void)
  {
    // TODO
  }
}
