
## 基本原理介绍

![[tof重建.excalidraw|800]]

## Speckled Image 设计

- 通过设计speckle point 之间的距离和旋转角度，最终保证 行方向上的 r 距离 > search range min + max
- 注意考虑投射器边缘存在畸变，会造成在边缘处的r变小
