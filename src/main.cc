#include "QGCApplication.h"
#include "QGCCommandLineParser.h"
#include "LogManager.h"
#include "QGCLoggingCategory.h"
#include "Platform.h"

#ifdef Q_OS_ANDROID
    #include <QtGui/QGuiApplication>
    #include <QtQml/QQmlApplicationEngine>
    #include <QtQuickControls2/QQuickStyle>
#endif

#ifdef QGC_UNITTEST_BUILD
    #include "UnitTestList.h"
#endif

QGC_LOGGING_CATEGORY_ON(MainLog, "Main")

int main(int argc, char *argv[])
{
#ifdef Q_OS_ANDROID
    // HACK: AX12 debugging - literal clone of window-test main(); no QGC code at all
    QGuiApplication cloneApp(argc, argv);
    QQuickStyle::setStyle(QStringLiteral("Basic"));
    QQmlApplicationEngine cloneEngine;
    cloneEngine.load(QUrl(QStringLiteral("qrc:/qml/QGroundControl/MainWindowStripped.qml")));
    return cloneApp.exec();
#endif

    // --- Parse command line arguments ---
    const auto args = QGCCommandLineParser::parse(argc, argv);
    if (const auto exitCode = QGCCommandLineParser::handleParseResult(args)) {
        return *exitCode;
    }

    // --- Platform initialization ---
    if (const auto exitCode = Platform::initialize(argc, argv, args)) {
        return *exitCode;
    }

    QGCApplication app(argc, argv, args);

    LogManager::installHandler(args.logOutput);

    Platform::setupPostApp();

#ifdef Q_OS_ANDROID
    // HACK: AX12 debugging - load stripped window immediately, before heavy init
    QQmlApplicationEngine earlyEngine;
    earlyEngine.load(QUrl(QStringLiteral("qrc:/qml/QGroundControl/MainWindowStripped.qml")));
#endif

    app.init();

    // Apply after installFilter() (called during app.init) so rules aren't overwritten.
    LogManager::applyEnvironmentLogLevel();

    // --- Run application or tests ---
    const auto run = [&]() -> int {
        using QGCCommandLineParser::AppMode;
        switch (QGCCommandLineParser::determineAppMode(args)) {
#ifdef QGC_UNITTEST_BUILD
        case AppMode::ListTests:
        case AppMode::Test:
            return QGCUnitTest::handleTestOptions(args);
#endif
        case AppMode::BootTest:
            if (!app.bootTestPassed()) {
                qCCritical(MainLog) << "Simple boot test failed";
                return EXIT_FAILURE;
            }
            qCInfo(MainLog) << "Simple boot test completed";
            return EXIT_SUCCESS;
        case AppMode::Gui:
            qCInfo(MainLog) << "Starting application event loop";
            return app.exec();
        }
        Q_UNREACHABLE();
    };

    const int exitCode = run();

    // --- Cleanup ---
    app.shutdown();

    qCInfo(MainLog) << "Exiting main";

    // Destroy LogManager while Qt is still fully functional (before static destruction).
    delete LogManager::instance();

    return exitCode;
}
