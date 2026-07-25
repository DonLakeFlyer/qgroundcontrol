# AX12 / Android Wrong-Window-Size Render Bug — Investigation State

**Status:** UNRESOLVED. Paused for later pickup.
**Branch:** this branch is a scratch/debugging branch full of temporary HACKs. Do **not** merge.

## The Bug

On a **RadioMaster AX12** (Android 9 / API 28, screen 1280x720 px, device-pixel-ratio 1.75)
QGroundControl renders its QML into only ~**57%** (`1/1.75`) of the screen, anchored to the
**bottom-left** (OpenGL origin), leaving white clear-color strips along the **top and right**.

- QML geometry bookkeeping is **correct**: `win` / `Screen` report `731x411` logical
  = `1280x720` physical at dpr `1.75`.
- Conclusion: the **render pipeline draws at dpr = 1.0** while the **window geometry uses
  dpr = 1.75**. The framebuffer/viewport is effectively half-scale.

A minimal reference app **`/Users/don/repos/window-test`** (a bare Qt Quick
`ApplicationWindow`) renders **fullscreen correctly** on the same device. The entire
investigation is a differential diff between QGC and window-test.

## What Has Been Exonerated (each verified by build + on-device screenshot = "still broken")

- `MainWindow.qml` (replaced with a stripped minimal window — still broken)
- `AA_ShareOpenGLContexts` attribute
- **All** QGC Java + JNI (`android/src/...`, `JNI_OnLoad` wrapped in `#if 0`)
- **Clone `main()`** — a bare `QGuiApplication` + `QQmlApplicationEngine` loading a trivial
  QML file, bypassing ALL QGC runtime, placed at the very top of `main()`. Still broken.
- QGC's custom `AndroidManifest.xml`
- **SDL3** (removed from build)
- **GStreamer** (`QGC_ENABLE_GST_VIDEOSTREAMING` OFF)
- The entire **`src/Android`** module (`QGC_NO_ANDROID_MODULE`)
- Android manifest / theme knobs (`extract_android_style=minimal`, splash, orientation)
- Android compat scaling meta-data
- **Gradle** configuration (AGP/Kotlin/Java versions, packagingOptions style excludes)
- The **`android/` resource overlay** — see below.

### Android packaging now byte-identical to window-test

QGC's `android/` directory was stripped to match window-test **exactly**:
`diff -r -x '.gradle' android/ ../window-test/android/` → **IDENTICAL**.
It now contains only `AndroidManifest.xml` (verbatim copy of window-test's) and
`res/drawable/splashscreen.xml`.

The **generated** manifests differ only in:
- injected app identity (package name, versionCode/Name, `android:label`, `lib_name` value)
- **auto-injected `<uses-permission>`** entries that androiddeployqt derives from the linked
  Qt modules (Bluetooth, Camera, Audio, etc.) — cannot be removed without changing the link set.

All **render-relevant** activity attributes are identical (`sensorLandscape`,
`extract_android_style=minimal`, splash meta-data, `configChanges`, `launchMode`, `exported`).

**Even with the Android packaging identical to window-test, the bug persists.**

## Remaining Prime Suspect

The **only** remaining variable is the **linked Qt-module / plugin / static-lib set** of
`libQGroundControl.so`, whose **global constructors run at `.so` load** (before any QGC code
in `main()` executes — which is why the clone `main()` is also broken).

- window-test links only `Qt6::Quick` + `Qt6::QuickControls2`.
- QGC links dozens of modules.

**Next step when resuming:** bisect `target_link_libraries` for the QGC target — progressively
strip toward `Qt6::Quick` + `Qt6::QuickControls2`, rebuilding until the render corrects.
First candidates to test: `Qt6::Widgets`, any QPA/platform plugin, `Qt6::OpenGL`, or a
Quick/RHI module that could alter QPA screen / RHI device-pixel-ratio at load time.

## Test Cycle

```bash
cd build/vs-code-android
adb uninstall org.mavlink.qgroundcontrol 2>/dev/null
adb install -r android-build/build/outputs/apk/debug/android-build-debug.apk
adb shell am force-stop org.mavlink.qgroundcontrol
adb shell am start -n org.mavlink.qgroundcontrol/org.qtproject.qt.android.bindings.QtActivity
sleep 30
adb exec-out screencap -p > /tmp/qgc.png   # then view the image
```

Build (let the **user** run this — bare `cmake .` reconfigure fails in the agent env with an
empty `CMAKE_PREFIX_PATH`):

```bash
cd build/vs-code-android && cmake . && ninja
```

## Active HACKs In This Branch (all must be reverted before any real merge)

**Android packaging (this session):**
- `android/` stripped to window-test parity (only `AndroidManifest.xml` +
  `res/drawable/splashscreen.xml`). Original QGC overlay backed up at
  `/tmp/qgc-android-overlay-backup/` (build.gradle, gradle.properties, gradlew(.bat),
  lint.xml, proguard-rules.pro, libs/, res/, assets/, src/, gradle/, and
  `AndroidManifest.xml.qgc`). **Note:** `/tmp` is volatile — original files are also
  recoverable from git history (they are tracked deletions).
- `cmake/platform/Android.cmake`: `QT_ANDROID_APP_ICON "@mipmap/ic_launcher"` line **removed**
  (mipmap resource no longer exists); `QT_ANDROID_PACKAGE_SOURCE_DIR` restored.

**src/main.cc:** clone `main()` (`QGuiApplication` + `QQuickStyle::setStyle("Basic")` +
`QQmlApplicationEngine` loading `MainWindowStripped.qml`) at top of `main()` under
`Q_OS_ANDROID`; plus a later unreached `earlyEngine` block.

**Stripped modules / guards (prior sessions):**
- SDL/Joystick removal: `src/Utilities/SDL/CMakeLists.txt`, `src/Utilities/CMakeLists.txt`,
  `src/Joystick/CMakeLists.txt`, `src/Joystick/JoystickManager.cc` (NullJoystickBackend),
  `test/Joystick` (disabled), `test/CMakeLists.txt`.
- `src/Android/AndroidInit.cc`: `JNI_OnLoad` wrapped in `#if 0`.
- `cmake/CustomOptions.cmake`: `QGC_ENABLE_GST_VIDEOSTREAMING` default OFF;
  `QGC_NO_ANDROID_MODULE` option added (ON).
- `src/CMakeLists.txt`: `QGC_NO_ANDROID_MODULE` guards `add_subdirectory(Android)`;
  `MainWindowStripped.qml` registered in QML_FILES.
- `src/Comms/CMakeLists.txt`, `SerialLink.h`, `QGCSerialPortInfo.h`: `QGC_NO_ANDROID_MODULE`
  guards + `Qt6::SerialPort` fallback.
- Consumer guards `#if defined(Q_OS_ANDROID) && !defined(QGC_NO_ANDROID_MODULE)` in:
  `src/Utilities/Platform/Platform.cc`, `src/Settings/AppSettings.cc`,
  `src/Vehicle/MultiVehicleManager.cc`, `src/Joystick/JoystickManager.cc`,
  `src/QmlControls/QGCFileDialogController.cc`.
- `src/QGCApplication.cc`: `JoystickManager::instance()->init()` wrapped `#ifndef Q_OS_ANDROID`.
- `src/API/QGCCorePlugin.cc`: `createRootWindow` load line commented (`Q_UNUSED(qmlEngine)`).
- `src/Utilities/Platform/Platform.cc`: `AA_ShareOpenGLContexts` guarded `&& !defined(Q_OS_ANDROID)`.
- `test/VideoManager/CMakeLists.txt`: `add_subdirectory(GStreamer)` behind the GST guard.
- `src/MainWindow/MainWindowStripped.qml` (NEW): minimal fullscreen window showing
  win/Screen/avail sizes.

## Key Environment Facts

- Qt **6.11.1** `android_arm64_v8a`. Kit `/Users/don/Qt/6.11.1/android_arm64_v8a`,
  host tools `/Users/don/Qt/6.11.1/macos`.
- Android build dir: `build/vs-code-android`.
- moccache (`QGC_USE_MOCCACHE`) keys on the full moc command line incl. `-D` defines —
  **avoid target-wide compile definitions** (they bust the entire cache → full rebuild).
