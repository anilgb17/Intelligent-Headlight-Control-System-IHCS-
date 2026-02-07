import pytest
from hypothesis import settings, Verbosity

# Configure Hypothesis to be less verbose by default but show all on failure
settings.register_profile("ci", max_examples=100, verbosity=Verbosity.normal)
settings.load_profile("ci")
