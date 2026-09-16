import sys

if "--std-backend" in sys.argv:
    sys.argv.remove("--std-backend")
    from ego_calibration.backends.std_runner import main
elif "--flash-backend" in sys.argv:
    sys.argv.remove("--flash-backend")
    from ego_calibration.backends.calibration_flash import main
elif "--desktop" in sys.argv:
    sys.argv.remove("--desktop")
    from ego_calibration.app import main
else:
    from ego_calibration.webapp import main


if __name__ == "__main__":
    raise SystemExit(main())
