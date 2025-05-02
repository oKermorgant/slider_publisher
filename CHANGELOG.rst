^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slider_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.4.1 (2025-05-02)
------------------
* add combobox for defalut value as list
* Contributors: Olivier Kermorgant

2.4.0 (2024-05-17)
------------------
* no need to specify message field if there is only one
* Contributors: Olivier Kermorgant

2.3.1 (2023-12-16)
------------------
* typo in readme
* Contributors: Olivier Kermorgant

2.3.0 (2023-12-16)
------------------
* text input
* remove dependency on numpy/scipy, use basic math to handle Quaternions
* can publish arrays of any type
* add rate in example
* Contributors: Olivier Kermorgant

2.2.1 (2022-09-24)
------------------
* remove py extension, fix rate default value
* switch to CMake to avoid deprecation messages
* Contributors: Olivier Kermorgant

2.2.0 (2022-09-21)
------------------
* add python3-scipy and numpy dependencies
* rate and config parameters
* Contributors: Olivier Kermorgant

2.1.1 (2022-05-03)
------------------
* underscores in setup.cfg
* Contributors: Olivier Kermorgant

2.1.0 (2022-03-18)
------------------
* do not re-call service if request has not changed
* example Twist defaults to 0 velocity
* Contributors: Olivier Kermorgant

2.0.1 (2021-08-19)
------------------
* bug in type detection for roll pitch yaw
* Contributors: Olivier Kermorgant

2.0.0 (2021-08-18)
------------------
* add service and detects value type (float / int / bool)
* can pick default value
* Contributors: Olivier Kermorgant

1.0.2 (2021-04-18)
------------------
* README typo
* Contributors: Olivier Kermorgant

1.0.0 (2020-06)
------------------
* ROS 2 branch

0.1.1 (2017-11-23)
------------------
* title is now node name
* Twist without default
* velocity -> twist
* Update README.md
* topic paths
* without python setup as its only an executable
* now displays topics in GUI
* works for arrays of float
* default timestamp if in message header
* more README
* initial commit
* Contributors: Olivier Kermorgant
