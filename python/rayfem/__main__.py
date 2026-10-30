"""``python -m rayfem``: the same as the ``rayfem`` command."""
import sys

from .cli import main

sys.exit(main())
