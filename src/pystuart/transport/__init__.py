"""
Hardware transport layer: frame codec, link backends, stream parsing.

Implemented: wire protocol codec (protocol.py) and the in-memory mock
link (mock.py) for tests and Bridge simulation. Planned: USB CDC /
serial backend for the Bridge board (sensor stream up, CMD_ACT down),
BLE reception and UDP output for the full-size rig.
See docs/pystuart_prototype_design.md, sections 8 and 11.
"""
