#!/usr/bin/env python3
"""Prefix each stdin line with a unix timestamp. The runner logs no times of its
own, so this is how a drill correlates its phases with the simulator's clock."""
import sys, time
for line in sys.stdin:
    sys.stdout.write(f"{time.time():.6f} {line}")
    sys.stdout.flush()
