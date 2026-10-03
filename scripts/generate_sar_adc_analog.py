#!/usr/bin/env python3
"""Generate and sign off the SKY130 SAR ADC analog layout blocks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


SCRIPT = Path(__file__).resolve()
GLAYOUT_REPO = SCRIPT.parents[1]
GLAYOUT_SRC = GLAYOUT_REPO / "src"
sys.path.insert(0, str(GLAYOUT_SRC))

from glayout.cells.composite.sar_adc import (  # noqa: E402
    clocked_dynamic_preamp,
    sar_bootstrapped_switch,
    sar_cdac_array,
    sar_cdac_switch_matrix,
    sar_sampling_frontend,
    sar_top_plate_reset,
    strongarm_comparator,
    strongarm_offset_trim,
)
from glayout.pdk.sky130_mapped import sky130_mapped_pdk  # noqa: E402


GENERATORS = {
    "clocked_dynamic_preamp": clocked_dynamic_preamp,
    "sar_bootstrapped_switch": sar_bootstrapped_switch,
    "sar_cdac_array": sar_cdac_array,
    "sar_cdac_switch_matrix": sar_cdac_switch_matrix,
    "sar_sampling_frontend": sar_sampling_frontend,
    "sar_top_plate_reset": sar_top_plate_reset,
    "strongarm_comparator": strongarm_comparator,
    "strongarm_offset_trim": strongarm_offset_trim,
}
RESOLUTION_DEPENDENT_BLOCKS = {
    "sar_cdac_array",
    "sar_cdac_switch_matrix",
}
DEFAULT_BLOCKS = (
    "sar_cdac_array",
    "sar_cdac_switch_matrix",
    "sar_sampling_frontend",
    "clocked_dynamic_preamp",
    "strongarm_comparator",
    "strongarm_offset_trim",
)
LEF_LAYERS = {
    "met1": "li1",
    "met2": "met1",
    "met3": "met2",
    "met4": "met3",
    "met5": "met4",
}


def _run(
    command: list[str], *, cwd: Path, log: Path, env: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    log.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(
            f"command failed ({result.returncode}): {' '.join(command)}; see {log}"
        )
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _tool_version(executable: str, *arguments: str) -> str | None:
    resolved = shutil.which(executable)
    if resolved is None:
        return None
    result = subprocess.run(
        [resolved, *arguments], capture_output=True, text=True, check=False
    )
    text = (result.stdout or result.stderr).strip().splitlines()
    return text[0] if text else resolved


def _gds_top_name(gds: Path) -> str:
    import klayout.db as kdb

    layout = kdb.Layout()
    layout.read(str(gds))
    tops = list(layout.top_cells())
    if len(tops) != 1:
        raise RuntimeError(f"expected one GDS top cell in {gds}, found {len(tops)}")
    return tops[0].name


def _write_lef(component, path: Path, macro_name: str) -> None:
    width = float(component.xmax - component.xmin)
    height = float(component.ymax - component.ymin)
    x_origin = float(component.xmin)
    y_origin = float(component.ymin)
    lines = [
        "VERSION 5.8 ;",
        'BUSBITCHARS "[]" ;',
        'DIVIDERCHAR "/" ;',
        f"MACRO {macro_name}",
        "  CLASS BLOCK ;",
        f"  ORIGIN {-x_origin:.6f} {-y_origin:.6f} ;",
        f"  SIZE {width:.6f} BY {height:.6f} ;",
        "  SYMMETRY X Y ;",
    ]
    pdk = sky130_mapped_pdk
    for name, port in sorted(component.ports.items()):
        glayer = pdk.layer_to_glayer(port.layer)
        lef_layer = LEF_LAYERS.get(glayer)
        if lef_layer is None:
            raise RuntimeError(f"no LEF layer mapping for {name} on {glayer}")
        x, y = map(float, port.center)
        half = max(float(port.width) / 2, 0.10)
        lines.extend(
            (
                f"  PIN {name}",
                "    DIRECTION INOUT ;",
                "    USE SIGNAL ;",
                "    PORT",
                f"      LAYER {lef_layer} ;",
                f"        RECT {x - half:.6f} {y - half:.6f} "
                f"{x + half:.6f} {y + half:.6f} ;",
                "    END",
                f"  END {name}",
            )
        )
    lines.extend((f"END {macro_name}", "END LIBRARY", ""))
    path.write_text("\n".join(lines), encoding="ascii")


def _drc(gds: Path, report: Path, log: Path) -> int:
    klayout = shutil.which("klayout")
    if klayout is None:
        raise RuntimeError("klayout is required for signoff")
    deck = GLAYOUT_SRC / "glayout/pdk/sky130_mapped/sky130.lydrc"
    _run(
        [
            klayout,
            "-b",
            "-r",
            str(deck),
            "-rd",
            f"input={gds}",
            "-rd",
            f"report={report}",
        ],
        cwd=gds.parent,
        log=log,
    )
    errors = len(ET.parse(report).getroot()[7])
    if errors:
        raise RuntimeError(f"{gds.stem}: KLayout DRC found {errors} errors")
    return errors


def _extract_and_lvs(
    *,
    gds: Path,
    reference: Path,
    reference_cell: str,
    pdk_root: Path,
    output_dir: Path,
) -> tuple[Path, Path]:
    magic = shutil.which("magic")
    netgen = shutil.which("netgen")
    if magic is None or netgen is None:
        raise RuntimeError("magic and netgen are required for signoff")
    magicrc = pdk_root / "sky130A/libs.tech/magic/sky130A.magicrc"
    setup = pdk_root / "sky130A/libs.tech/netgen/sky130A_setup.tcl"
    for required in (magicrc, setup):
        if not required.is_file():
            raise RuntimeError(f"missing SKY130 signoff file: {required}")

    extracted = output_dir / f"{reference_cell}.extracted.spice"
    magic_script = output_dir / f"{reference_cell}.extract.tcl"
    gds_top = _gds_top_name(gds)
    magic_script.write_text(
        "\n".join(
            (
                f"gds read {{{gds}}}",
                f"load {{{gds_top}}}",
                "select top cell",
                "port makeall",
                "extract do local",
                "extract all",
                "ext2spice lvs",
                f"ext2spice -o {extracted}",
                "quit -noprompt",
                "",
            )
        ),
        encoding="ascii",
    )
    _run(
        [magic, "-rcfile", str(magicrc), "-noconsole", "-dnull", str(magic_script)],
        cwd=output_dir,
        log=output_dir / f"{reference_cell}.magic.log",
    )
    if not extracted.is_file():
        raise RuntimeError(f"Magic did not produce {extracted}")

    lvs_report = output_dir / f"{reference_cell}.lvs.out"
    _run(
        [
            netgen,
            "-batch",
            "lvs",
            f"{extracted} {gds_top}",
            f"{reference} {reference_cell}",
            str(setup),
            str(lvs_report),
        ],
        cwd=output_dir,
        log=output_dir / f"{reference_cell}.netgen.log",
    )
    report_text = lvs_report.read_text(encoding="utf-8")
    if "Circuits match uniquely." not in report_text:
        raise RuntimeError(f"{reference_cell}: LVS failed; see {lvs_report}")
    return extracted, lvs_report


def _git_revision() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=GLAYOUT_REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def generate(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    pdk_root = args.pdk_root.resolve() if args.pdk_root else None
    if not args.no_signoff and pdk_root is None:
        raise RuntimeError(
            "--pdk-root (or PDK_ROOT) is required unless --no-signoff is used"
        )

    manifest: dict[str, object] = {
        "schema_version": 1,
        "resolution_bits": args.bits,
        "glayout_revision": _git_revision(),
        "pdk_root": str(pdk_root) if pdk_root else None,
        "tools": {
            "python": sys.version.split()[0],
            "klayout": _tool_version("klayout", "-v"),
            "magic": _tool_version("magic", "--version"),
            "netgen": _tool_version("netgen", "-batch", "version"),
        },
        "blocks": {},
    }

    for name in args.blocks:
        if name in RESOLUTION_DEPENDENT_BLOCKS:
            generator_options = {"bits": args.bits}
        elif name == "strongarm_comparator":
            generator_options = {
                "device_scale": args.comparator_device_scale,
                "input_pair_scale": args.comparator_input_pair_scale,
                "input_pair_length_scale": args.comparator_input_pair_length_scale,
            }
        elif name == "clocked_dynamic_preamp":
            generator_options = {
                "input_width_um": args.preamp_input_width_um,
                "load_width_um": args.preamp_load_width_um,
                "feedback_width_um": args.preamp_feedback_width_um,
                "reset_width_um": args.preamp_reset_width_um,
                "tail_width_um": args.preamp_tail_width_um,
                "length_um": args.preamp_length_um,
            }
        elif name == "strongarm_offset_trim":
            generator_options = {
                "bits": args.trim_bits,
                "unit_width_um": args.trim_unit_width_um,
                "selector_length_um": args.trim_selector_length_um,
                "footer_width_um": args.trim_footer_width_um,
                "footer_length_um": args.trim_footer_length_um,
            }
        else:
            generator_options = {}
        component = GENERATORS[name](sky130_mapped_pdk, **generator_options)
        output_name = (
            f"{name}_{args.bits}bit"
            if args.bits != 10 and name in RESOLUTION_DEPENDENT_BLOCKS
            else name
        )
        gds = output_dir / f"{output_name}.gds"
        functional = output_dir / f"{output_name}.spice"
        lvs_reference = output_dir / f"{output_name}.lvs.spice"
        lef = output_dir / f"{output_name}.lef"
        component.write_gds(gds)
        functional.write_text(
            component.info["netlist"].generate_netlist() + "\n", encoding="ascii"
        )
        lvs_reference.write_text(
            component.info["lvs_netlist"].generate_netlist() + "\n", encoding="ascii"
        )
        _write_lef(component, lef, output_name)

        block_manifest: dict[str, object] = {
            "gds_top": _gds_top_name(gds),
            "bbox_um": [
                float(component.xmin),
                float(component.ymin),
                float(component.xmax),
                float(component.ymax),
            ],
            "ports": sorted(component.ports),
            "generator_info": {
                key: value
                for key, value in component.info.items()
                if key not in ("netlist", "lvs_netlist", "mapping")
            },
            "files": {},
        }
        files = [gds, functional, lvs_reference, lef]
        if not args.no_signoff:
            drc_report = output_dir / f"{output_name}.drc.lyrdb"
            drc_errors = _drc(
                gds, drc_report, output_dir / f"{output_name}.klayout.log"
            )
            extracted, lvs_report = _extract_and_lvs(
                gds=gds,
                reference=lvs_reference,
                reference_cell=output_name,
                pdk_root=pdk_root,
                output_dir=output_dir,
            )
            files.extend((drc_report, extracted, lvs_report))
            block_manifest["signoff"] = {
                "klayout_drc_errors": drc_errors,
                "netgen_lvs": "match_unique",
            }
        else:
            block_manifest["signoff"] = "not_run"

        block_manifest["files"] = {
            path.name: {"sha256": _sha256(path), "bytes": path.stat().st_size}
            for path in files
        }
        manifest["blocks"][output_name] = block_manifest

    manifest_path = output_dir / "analog_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(manifest_path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="generated artifact directory"
    )
    parser.add_argument(
        "--pdk-root",
        type=Path,
        default=Path(os.environ["PDK_ROOT"]) if os.getenv("PDK_ROOT") else None,
    )
    parser.add_argument(
        "--bits",
        type=int,
        choices=(8, 10),
        default=10,
        help="CDAC and switch-matrix resolution (default: 10)",
    )
    parser.add_argument(
        "--preamp-input-width-um", type=float, default=2.0
    )
    parser.add_argument(
        "--preamp-load-width-um", type=float, default=2.0
    )
    parser.add_argument("--preamp-feedback-width-um", type=float, default=1.0)
    parser.add_argument("--preamp-reset-width-um", type=float, default=1.0)
    parser.add_argument(
        "--preamp-tail-width-um", type=float, default=1.0
    )
    parser.add_argument("--preamp-length-um", type=float, default=0.5)
    parser.add_argument(
        "--comparator-device-scale",
        type=int,
        default=1,
        help="integer width/finger scale for every StrongARM functional device",
    )
    parser.add_argument(
        "--comparator-input-pair-scale",
        type=int,
        default=None,
        help="optional StrongARM input-pair-only width/finger scale",
    )
    parser.add_argument(
        "--comparator-input-pair-length-scale",
        type=int,
        default=1,
        help="optional StrongARM input-pair-only channel-length scale",
    )
    parser.add_argument("--trim-bits", type=int, default=4)
    parser.add_argument("--trim-unit-width-um", type=float, default=0.42)
    parser.add_argument("--trim-selector-length-um", type=float, default=1.2)
    parser.add_argument("--trim-footer-width-um", type=float, default=8.0)
    parser.add_argument("--trim-footer-length-um", type=float, default=0.15)
    parser.add_argument(
        "--blocks",
        nargs="+",
        choices=tuple(GENERATORS),
        default=DEFAULT_BLOCKS,
    )
    parser.add_argument(
        "--no-signoff",
        action="store_true",
        help="generate GDS/netlists/LEF without DRC, extraction, or LVS",
    )
    return parser


if __name__ == "__main__":
    generate(_parser().parse_args())
