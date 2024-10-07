# common_utils
## ファイル構成
pcl_utility :PCL関連の関数ラッパー
ros2_utility :ros2関連の関数ラッパー
utility :数学,文字列操作,単位変換など
ext : その他拡張ライブラリ等

## include
```C++
#define USE_PCL // pcl関連のツールを使う場合
#define USE_ROS2 // ros2関連のツールを使う場合
#define USE_JSK // jskプラグイン関連のツールを使う場合
#define USE_OPENCV // OoenCV関連のツールを使う場合
#include "common_utils/common_utils.hpp"
```
## namespace
```C++
using namespace common_utils; // 共通ツールのnamespace
using namespace pcl_utils; // PCLツールのnamespace
using namespace ros2_utils; // ROS2ツールのnamespace
using namespace ext; // 拡張ライブラリのnamespace
```