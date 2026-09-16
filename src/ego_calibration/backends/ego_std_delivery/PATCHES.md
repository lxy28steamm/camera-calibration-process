# Included Kalibr Patches

`kalibr_source/` is based on Kalibr commit
`1f60227442d25e36365ef5f72cd80b9666d73467`. The following patches are already
applied to that source snapshot and are retained in `patches/` for auditability:

- `patches/kalibr_calibrate_cameras_fixed_corner_filter.patch`
- `patches/numpy_eigen_skip_test_extension.patch`

`kalibr_calibrate_cameras_fixed_corner_filter.patch` adds fixed-threshold
corner reprojection filtering and the options `--max-corner-reproj-px` and
`--max-corner-filter-rounds`. The demo workflow uses those options, so a stock
upstream Kalibr checkout is not interchangeable with this source snapshot.

`numpy_eigen_skip_test_extension.patch` disables the uninstalled
`numpy_eigen_test` extension by default. That extension is only consumed by
the package's own tests and adds hundreds of generated translation units; it is
not required by the calibration runtime.
