"""Compatibility entrypoint for final-production verification.
Historical wrapper-only C fixtures are retained as provenance; their invariants
now run against the integrated production code in test/integration_cases.h.
Use tools/run_digital.py and tools/run_migrated_contracts.py for current flags.
"""
import pathlib,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1]
raise SystemExit(subprocess.call([sys.executable,str(root/'tools'/'run_digital.py'),'numeric_and_imd_contract',*sys.argv[1:]],cwd=root))
