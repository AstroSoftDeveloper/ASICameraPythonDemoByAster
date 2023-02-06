# ZWO_ASICamera

作者：asterjian@qq.com



提供 ASICamera 的 Python 绑定代码

## 编译：

```bash
> cmake -B build -S .
> cmake --build . --config Release
```

## 目录说明：

### 3rdparty

第三方库目录，包含 pybind11 库；



### ASI_SDK

ASI_SDK Windows SDK 代码



### ASICamera

ASICamera 的 Python 绑定代码

- TestFunction.py  

  相机Python API的测试代码

- TestCamera.py

  相机和OpenCV结合的测试代码

