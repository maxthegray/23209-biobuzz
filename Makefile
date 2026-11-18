# shortcuts for working with the robot from the terminal. run `make` to see them all.
# Android Studio still works exactly like before, this is just faster.

HUB_IP   ?= 192.168.43.1
HUB_PORT ?= 5555
GRADLE   ?= ./gradlew
export HUB_IP HUB_PORT GRADLE

# gradle 8.9 can't run on the Mac's default Java (24), so use the same JDK 21 Android Studio uses.
# if there isn't one here (CI), whatever JAVA_HOME is already set stays
JDK21 := $(shell /usr/libexec/java_home -v 21 2>/dev/null)
ifeq ($(JDK21),)
  JDK21 := $(wildcard /Library/Java/JavaVirtualMachines/jdk-21.jdk/Contents/Home)
endif
ifneq ($(JDK21),)
  export JAVA_HOME := $(JDK21)
endif

# `make pull-logs 5` passes 5 as the count (`make pull-logs N=5` works too)
ifeq (pull-logs,$(firstword $(MAKECMDGOALS)))
  N ?= $(word 2,$(MAKECMDGOALS))
  $(eval $(word 2,$(MAKECMDGOALS)):;@:)
endif

.DEFAULT_GOAL := help

help: ## Show this list
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# stops with a clear message when there's no usable Java
java:
	@if [ -z "$$JAVA_HOME" ] || [ ! -x "$$JAVA_HOME/bin/java" ]; then \
		echo "error: can't find JDK 21. install it, or set JAVA_HOME to a JDK 17-21"; exit 1; fi

## --- build ---------------------------------------------------------------

build: java ## Compile everything, don't touch the robot
	$(GRADLE) :TeamCode:assembleDebug

test: java ## Run the tests that work without a robot (robot code, then the laptop tools)
	$(GRADLE) :TeamCode:testDebugUnitTest
	python3 -m unittest tools/logs/test_analyze_wpilog.py tools/logs/test_tuned.py \
		tools/buzzscope/test_log_viewer.py tools/buzzscope/test_traction.py tools/buzzscope/test_battery.py
	@if command -v node >/dev/null 2>&1; then node --test tools/buzzscope/viewer/core.test.mjs; \
	else echo "node isn't installed, skipping the BuzzScope browser-math tests"; fi

## --- robot ---------------------------------------------------------------

deploy: java ## Put the code on the robot: hot reload if that's enough, full install if not
	@tools/deploy/deploy.sh auto

check-deploy: ## Say whether deploy would hot reload or full install, change nothing
	@tools/deploy/deploy.sh check

install: java ## Full install, no questions asked (about a minute)
	@tools/deploy/deploy.sh full

hot: java ## Hot reload, no questions asked (about a second, teamcode Java only)
	@tools/deploy/deploy.sh hot

unsloth: java ## Remove hot-reloaded code from the hub, so the installed app runs as installed
	@ANDROID_SERIAL="$$(tools/hub-connect.sh $(HUB_IP) $(HUB_PORT))" $(GRADLE) removeSlothRemote

connect: ## Wait until the hub is reachable (Wi-Fi first, then USB)
	@tools/hub-connect.sh $(HUB_IP) $(HUB_PORT) >/dev/null && echo "connected"

scrcpy: ## Show the hub's screen on the laptop (needs scrcpy installed)
	@command -v scrcpy >/dev/null 2>&1 || { echo "scrcpy isn't installed (brew install scrcpy)"; exit 1; }
	@ANDROID_SERIAL="$$(tools/hub-connect.sh $(HUB_IP) $(HUB_PORT))" scrcpy

## --- logs ----------------------------------------------------------------

pull-logs: ## Copy the newest logs off the hub into robot-logs/ (`make pull-logs 5`, default all 30)
	@tools/logs/pull-logs.sh "$(N)" $(HUB_IP) $(HUB_PORT)

analyze: ## Print a summary of the newest log in robot-logs/
	@python3 tools/logs/analyze_wpilog.py

viewer: ## Open BuzzScope at http://127.0.0.1:8008 to replay the logs in robot-logs/
	python3 tools/buzzscope/log_viewer.py

tuned: ## List the numbers changed live in Panels and the line each one goes on (newest log)
	@python3 tools/logs/tuned.py

.PHONY: help java build test deploy check-deploy install hot unsloth connect scrcpy pull-logs analyze viewer tuned
