@echo off
javac -Xlint -classpath "%WEBOTS_HOME%\lib\controller\java\Controller.jar";robotcore;lib\Jamepad.jar;src;. src\FTCController.java -d bin > compile.log 2> compile.err
copy src\telemetry.html bin
ant -f build_webots_controller.xml