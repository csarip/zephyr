#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Build-time negative tests of the TMC5262 driver bindings and BUILD_ASSERTs, and the symbol
# inspection of TC-VAR-018. Twister has no "expected failure" for a build, so these run as a
# script. Each case adds one chip node to the build_all stepper test.
#
# A negative case passes only if the build FAILS and the log contains the one specific error
# that the case is about (a BUILD_ASSERT text, or the devicetree message naming the property, the
# node and the binding). Before any negative case runs, positive controls (a valid chip node of
# every variant and the valid twins of the clock and SPI frequency cases) must build: if one does
# not, the script aborts, because a failing control would make every negative case pass for the
# wrong reason.
#
# Cases (STS ids): TC-INIT-004, TC-INIT-012, TC-VAR-018, TC-BUS-002, TC-CONFIG-005,
# TC-CONFIG-012, TC-CONFIG-015, TC-FEATURE-013.
#
# Usage: tests/drivers/stepper/tmc5262/negative/run.sh [output-directory]
# Needs the west environment (source the virtual environment first).

set -u
ZEPHYR_BASE="${ZEPHYR_BASE:-$(cd "$(dirname "$0")/../../../../.." && pwd)}"
OUT="${1:-/tmp/tmc5262-negative}"
APP="$ZEPHYR_BASE/tests/drivers/build_all/stepper"
BOARD="native_sim/native/64"
pass=0
fail=0

mkdir -p "$OUT"

# make_overlay <file> <compatible> <chip properties> [extra root nodes]
make_overlay() {
	cat > "$1" <<EOF
/ { ${4:-} };
&test_spi {
	neg_chip: neg-chip@7 {
		compatible = "$2";
		status = "okay";
		reg = <0x07>;
		spi-max-frequency = <8000000>;
		$3
	};
};
EOF
}

# build <id> <overlay>: returns the exit status of the build, the log is $OUT/<id>.log
build() {
	local dir="$OUT/$1"

	rm -rf "$dir"
	west build -b "$BOARD" "$APP" -d "$dir" -- -DEXTRA_DTC_OVERLAY_FILE="$2" > "$dir.log" 2>&1
}

# run_ok <id> <compatible> <properties> [extra root nodes]: a positive control, must build
run_ok() {
	make_overlay "$OUT/$1.overlay" "$2" "$3" "${4:-}"
	if build "$1" "$OUT/$1.overlay"; then
		echo "PASS control $1"
		pass=$((pass + 1))
	else
		echo "ABORT: the positive control $1 does not build ($OUT/$1.log)"
		exit 2
	fi
}

# run_case <id> <message regex> <compatible> <properties> [extra root nodes]
run_case() {
	local id="$1" pattern="$2"

	make_overlay "$OUT/$id.overlay" "$3" "$4" "${5:-}"
	if build "$id" "$OUT/$id.overlay"; then
		echo "FAIL $id: the build succeeded"
		fail=$((fail + 1))
	elif grep -Eq "$pattern" "$OUT/$id.log"; then
		echo "PASS $id"
		pass=$((pass + 1))
	else
		echo "FAIL $id: failed with another error than '$pattern' ($OUT/$id.log)"
		fail=$((fail + 1))
	fi
}

# Message patterns. The devicetree messages name the property, the node and the binding.
undeclared() { # <property> <variant>
	echo "'$1' appears in [^ ]*neg-chip@7 .*but is not declared in 'properties:' in [^ ]*adi,$2\.yaml"
}
not_in_enum() { # <property> <value> <variant>
	echo "value of property '$1' on [^ ]*neg-chip@7 .*\('$2'\) is not in 'enum' list in [^ ]*adi,$3\.yaml"
}
assertion() { # <text>
	echo "static assertion failed: \"$1"
}

# ---- positive controls
run_ok control_tmc5262 "adi,tmc5262" ""
run_ok control_tmc2262 "adi,tmc2262" ""
run_ok control_tmc3241 "adi,tmc3241" ""
run_ok control_clock_16MHz "adi,tmc5262" 'clock-source = "external"; clock-frequency = <16000000>;'
run_ok control_clock_1MHz "adi,tmc5262" 'clock-source = "external"; clock-frequency = <1000000>;'
run_ok control_clock_32MHz "adi,tmc3241" 'clock-source = "external"; clock-frequency = <32000000>;'
run_ok control_spi_8MHz "adi,tmc2262" 'spi-max-frequency = <8000000>;'
run_ok control_ramp_props "adi,tmc3241" 'stop-l-enable; virtual-stop-l = <(-5)>;'
run_ok control_udc "adi,tmc5262" 'en-stealthchop; en-softstop; udc-enable;'
run_ok control_do_xcomp "adi,tmc3241" \
	'do0-gpios = <&test_gpio 4 GPIO_ACTIVE_HIGH>; do0-sources = "xcomp";'
run_ok control_do_udcstep_2262 "adi,tmc2262" 'do1-sources = "udcstep";'
run_ok control_do_step_5262 "adi,tmc5262" 'do0-sources = "step";'
run_ok control_hyst_3241 "adi,tmc3241" 'hyst = <1>;'
run_ok control_hend_5262 "adi,tmc5262" 'hend-offset = <1>;'
run_ok control_scale_ok "adi,tmc5262" 'current-range-scale-percent = <50>; current-range-ma = <1060>;'
run_ok control_step_gpios "adi,tmc2262" "" \
	'ok_ctrl: ok-ctrl { compatible = "zephyr,gpio-step-dir-stepper-ctrl";
	step-gpios = <&test_gpio 0 0>; dir-gpios = <&test_gpio 0 0>;
	stepper-driver = <&adi_tmc2262_stepper_driver>; };'

# ---- TC-INIT-004: the STEP/DIR controller of the TMC2262 needs its step pin
run_case init004_step_gpios 'DT_N_S_bad_ctrl_P_step_gpios[A-Za-z0-9_]*. undeclared' \
	"adi,tmc2262" "" \
	'bad_ctrl: bad-ctrl { compatible = "zephyr,gpio-step-dir-stepper-ctrl";
	dir-gpios = <&test_gpio 0 0>; stepper-driver = <&adi_tmc2262_stepper_driver>; };'

# ---- TC-INIT-012: external clock outside 1..32 MHz in 1 MHz steps
for c in tmc5262:500000 tmc5262:33000000 tmc5262:16500000 tmc2262:500000 tmc3241:33000000; do
	run_case "init012_${c%%:*}_${c##*:}" \
		"$(assertion 'TMC5262: external clock must be 1\.\.32 MHz in 1 MHz steps')" \
		"adi,${c%%:*}" "clock-source = \"external\"; clock-frequency = <${c##*:}>;"
done

# ---- TC-BUS-002: SPI clock above 8 MHz
for v in tmc5262 tmc2262 tmc3241; do
	run_case "bus002_${v}_8000001" \
		"$(assertion 'TMC5262: spi-max-frequency must not exceed 8 MHz')" "adi,$v" \
		'spi-max-frequency = <8000001>;'
done
run_case bus002_tmc5262_10000000 \
	"$(assertion 'TMC5262: spi-max-frequency must not exceed 8 MHz')" "adi,tmc5262" \
	'spi-max-frequency = <10000000>;'

# ---- TC-CONFIG-005: chopper fields of another variant
run_case config005_3241_hend "$(undeclared hend-offset tmc3241)" "adi,tmc3241" 'hend-offset = <1>;'
run_case config005_5262_hyst "$(undeclared hyst tmc5262)" "adi,tmc5262" 'hyst = <1>;'
run_case config005_2262_hyst "$(undeclared hyst tmc2262)" "adi,tmc2262" 'hyst = <1>;'
run_case config005_2262_disfdcc "$(undeclared disfdcc tmc2262)" "adi,tmc2262" 'disfdcc;'

# ---- TC-CONFIG-012: DO source the variant does not provide
run_case config012_2262_step "$(not_in_enum do0-sources step tmc2262)" "adi,tmc2262" \
	'do0-sources = "step";'
run_case config012_3241_udcstep "$(not_in_enum do1-sources udcstep tmc3241)" "adi,tmc3241" \
	'do1-sources = "udcstep";'

# ---- TC-CONFIG-015: options that a variant does not have
run_case config015_3241_stealth "$(undeclared en-stealthchop tmc3241)" "adi,tmc3241" \
	'en-stealthchop;'
run_case config015_2262_step_source "$(undeclared step-source tmc2262)" "adi,tmc2262" \
	'step-source = "step-dir";'
run_case config015_3241_sd_on_meas "$(undeclared sd-on-meas-hi tmc3241)" "adi,tmc3241" \
	'sd-on-meas-hi = <3>;'

# ---- TC-FEATURE-013: properties of the other variants
run_case feature013_2262_vmax "$(undeclared vmax tmc2262)" "adi,tmc2262" 'vmax = <100>;'
run_case feature013_3241_stealth "$(undeclared en-stealthchop tmc3241)" "adi,tmc3241" \
	'en-stealthchop;'
run_case feature013_3241_udc "$(undeclared udc-enable tmc3241)" "adi,tmc3241" 'udc-enable;'

# ---- build-time property checks (FR-INIT-003, FR-CONFIG-001, -010, FR-FEATURE-006)
run_case check_scale_range \
	"$(assertion 'TMC5262: current-range-scale-percent below 100 requires the lowest range')" \
	"adi,tmc5262" 'current-range-scale-percent = <50>; current-range-ma = <2120>;'
run_case check_toff_zero "$(assertion 'TMC5262: toff = 0 disables the driver')" "adi,tmc5262" \
	'toff = <0>;'
run_case check_drv_enn_polarity \
	"$(assertion 'TMC5262: drv-enn-gpios must be GPIO_ACTIVE_LOW')" "adi,tmc5262" \
	'drv-enn-gpios = <&test_gpio 3 GPIO_ACTIVE_HIGH>;'
run_case check_ovn_encoder \
	"$(assertion 'TMC5262: ENCN cannot be overvoltage output and encoder N channel')" \
	"adi,tmc5262" 'ov-output; encoder-n-channel;'
run_case check_do_without_source "$(assertion 'TMC5262: do0-gpios needs do0-sources')" \
	"adi,tmc5262" 'do0-gpios = <&test_gpio 4 GPIO_ACTIVE_HIGH>;'
run_case check_udc_no_stealth \
	"$(assertion 'TMC5262: udc-enable needs en-stealthchop')" "adi,tmc2262" 'udc-enable;'
run_case check_udc_no_softstop "$(assertion 'TMC5262: udc-enable needs en-softstop')" \
	"adi,tmc5262" 'en-stealthchop; udc-enable;'
run_case check_switch_step_dir \
	"$(assertion 'TMC5262: switch and virtual stop properties need step-source = ramp-generator')" \
	"adi,tmc5262" 'step-source = "step-dir"; stop-l-enable;'

# ---- TC-VAR-018: variant code is excluded from a build that has no node of the variant. Build
# the sample for a single variant and inspect the compiled objects (the linker would drop
# unreferenced functions of the image anyway).
NM="$(find "$HOME" -maxdepth 5 -name arm-zephyr-eabi-nm 2>/dev/null | head -1)"
SAMPLE="$ZEPHYR_BASE/samples/drivers/stepper/tmc5262"
if [ -n "$NM" ]; then
	for v in tmc2262 tmc3241; do
		d="$OUT/var018_$v"
		rm -rf "$d"
		if ! west build -b nucleo_g071rb "$SAMPLE" -d "$d" -- \
			-DEXTRA_DTC_OVERLAY_FILE="$v.overlay" > "$d.log" 2>&1; then
			echo "FAIL var018_$v: the sample did not build ($d.log)"
			fail=$((fail + 1))
			continue
		fi
		syms="$(find "$d" -name "tmc5262*.c.obj" -exec "$NM" {} +)"
		if [ "$v" = tmc2262 ]; then
			absent='tmc5262_variant_tmc3241|tmc5262_variant_tmc5262|tmc5262_set_switch_config|tmc5262_stepper_ctrl'
			present='tmc5262_variant_tmc2262|tmc5262_set_stealthchop'
		else
			absent='tmc5262_variant_tmc2262|tmc5262_variant_tmc5262|tmc5262_set_stealthchop|tmc5262_set_udcstep|tmc5262_measure_rcoil|tmc5262_set_stallguard_plus|tmc5262_set_coolstep_plus'
			present='tmc5262_variant_tmc3241|tmc5262_set_switch_config'
		fi
		if echo "$syms" | grep -Eq "$absent"; then
			echo "FAIL var018_$v: symbols of other variants are present"
			fail=$((fail + 1))
		elif ! echo "$syms" | grep -Eq "$present"; then
			echo "FAIL var018_$v: the symbols of the variant itself are missing"
			fail=$((fail + 1))
		else
			echo "PASS var018_$v ($(stat -c %s "$d/zephyr/zephyr.bin") bytes image)"
			pass=$((pass + 1))
		fi
	done
else
	echo "SKIP TC-VAR-018: arm-zephyr-eabi-nm not found"
fi

echo "negative tests: $pass passed (controls included), $fail failed"
[ "$fail" -eq 0 ]
