# Camera Intrinsics calibration tools

Set of tools allowing intrinsic calibration of cameras. Currently supported models: pinhole and Kannala and Brandt fisheye.

The code is based on the paper:

K. Kanatani, "Calibration of Ultrawide Fisheye Lens Cameras by Eigenvalue Minimization", *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 35, No. 4, 2013

### Build dependencies

The project uses Eigen3 and OpenCV (tested with 4.6).

## Usage

### Compiling patterns

Given set of PNG images named `<prefix>NNN` (where `<prefix>` may include folder path and `NNN` are `0`-based, `0`-padded three-digit consecutive integers), of the same size and containing chessboard, the `compile-patterns` tool prints preprocessed patterns to the standard output (and some relevant information to standard error output). General usage is

```
compile-patterns <prefix> <pattern width> <pattern height>
```

Images need not to be grayscale.

For example, given image files with 9x6 chessboard
```
images/capture000.png
images/capture001.png
images/capture002.png
...
images/capture052.png

```
one can compile patterns into `patterns.txt` file using following command:
```
$ ./compile-patterns images/capture 9 6 > patterns.txt
```

### Calibration from scratch

To calibrate a camera one first needs to precompile patterns (section 'Compiling patterns') to a text file and to know approximate field of view for the camera, in degrees (for fisheye cameras it should be safe to assume 180 degree field of view). For precompiled patterns in `patterns.txt`, to calibrate fisheye camera just run
```
$ ./calibrate patterns.txt 180
```
At the end, the tool prints parameters (after `Output: `) in order `mu`, `mv`, `u0`, `v0`, `k2`, `k3`, `k4`, `k5`.

### Correcting parameters

Given approximate calibration parameters `mu`, `mv`, `u0`, `v0`, `k2`, `k3`, `k4`, `k5`, it is possible to recalibrate using
```
correct <precompiled patterns> <mu> <mv> <u0> <v0> <k2> <k3> <k4> <k5>
```

Quite good effects can be achieved after re-running `correct` tool with `k2`, `k3`, `k4` and `k5` set to `0`.

### Checking calibration parameters

To check the quality of parameters one runs
```
intrinsics-test <precompiled patterns> <mu> <mv> <u0> <v0> <k2> <k3> <k4> <k5>
```

It outputs value of error function: the closer to `0`, the better.
