"""Middleware selection for the wired IVN.

Every node that talks to a ZCU has to be pinned to the wired interface and, for
the image nodes, to a fixed participant index. Fast DDS expresses that in an XML
profile named by FASTRTPS_DEFAULT_PROFILES_FILE; Cyclone DDS expresses the same
thing in its own XML named by CYCLONEDDS_URI. This module is the single place
that knows which of the two is active and how to name its profile, so launch
files and the GUI process manager stay middleware-neutral.

Read the committed common mode, or use AUTOSDV_RMW for legacy selection
(fastdds|cyclonedds|zenoh|zenoh_bridge|zenoh_bridge_pico; default fastdds).

zenoh_bridge is the odd one out: the nodes themselves run on Cyclone DDS, but
confined to the loopback interface, and zenoh-bridge-ros2dds carries their
traffic to the ZCUs over zenoh.  So it selects rmw_cyclonedds_cpp with the
single cyclonedds_bridge_local.xml profile rather than a per-node one.

Profiles come in sets whose names differ only by prefix (and, for zenoh, by
suffix), for example
    fastdds_image_03_participant22.xml
    cyclonedds_image_03_participant22.xml
    zenoh_image_03_participant22.json5
so callers keep referring to the Fast DDS name and this module translates it.

Zenoh additionally needs a router; see scripts/run_zenoh_router.sh.
zenoh_bridge needs the two bridges; see scripts/run_zenoh_bridge.sh.
"""

import os

from .communication_mode import selected_mode, state

FASTDDS = 'fastdds'
CYCLONEDDS = 'cyclonedds'
ZENOH = 'zenoh'
ZENOH_BRIDGE = 'zenoh_bridge'
ZENOH_BRIDGE_PICO = 'zenoh_bridge_pico'

_RMW_IMPLEMENTATIONS = {
    FASTDDS: 'rmw_fastrtps_cpp',
    CYCLONEDDS: 'rmw_cyclonedds_cpp',
    ZENOH: 'rmw_zenoh_cpp',
    # The bridge speaks DDS to the nodes; only the wire is zenoh.
    ZENOH_BRIDGE: 'rmw_cyclonedds_cpp',
    ZENOH_BRIDGE_PICO: 'rmw_cyclonedds_cpp',
}

# The shared wired-IVN profile is the one pair that is not a simple rename.
_DEFAULT_PROFILE = {
    FASTDDS: 'fastdds_whitelist.xml',
    CYCLONEDDS: 'cyclonedds_ivn.xml',
    ZENOH: 'zenoh_session_ivn.json5',
    ZENOH_BRIDGE: 'cyclonedds_bridge_local.xml',
    ZENOH_BRIDGE_PICO: 'cyclonedds_bridge_local.xml',
}

# Zenoh configs are JSON5 rather than XML, so the rename also changes suffix.
_PROFILE_SUFFIX = {
    FASTDDS: '.xml',
    CYCLONEDDS: '.xml',
    ZENOH: '.json5',
    ZENOH_BRIDGE: '.xml',
    ZENOH_BRIDGE_PICO: '.xml',
}


def middleware():
    """Return the selected middleware, validating AUTOSDV_RMW."""
    saved = selected_mode()
    selected = os.environ.get('AUTOSDV_RMW', saved).strip().lower()
    if state() and selected != saved:
        raise RuntimeError(f'AUTOSDV_RMW={selected} conflicts with selected mode {saved}')
    if selected not in _RMW_IMPLEMENTATIONS:
        raise RuntimeError(
            f'AUTOSDV_RMW must be one of {sorted(_RMW_IMPLEMENTATIONS)}, '
            f'got {selected!r}')
    return selected


def rmw_implementation():
    """Return the RMW_IMPLEMENTATION value for the selected middleware."""
    return _RMW_IMPLEMENTATIONS[middleware()]


def profile_filename(fastdds_filename):
    """Translate a Fast DDS profile file name to the selected middleware's."""
    selected = middleware()
    if selected == FASTDDS:
        return fastdds_filename
    if fastdds_filename == _DEFAULT_PROFILE[FASTDDS]:
        return _DEFAULT_PROFILE[selected]
    if not fastdds_filename.startswith('fastdds_'):
        raise RuntimeError(
            f'cannot derive a {selected} profile from {fastdds_filename!r}')
    if selected in (ZENOH_BRIDGE, ZENOH_BRIDGE_PICO):
        # No DDS packet leaves the host in this environment, so there is no
        # interface to whitelist and no participant index to pin: every node
        # shares the one loopback profile.  The name is still validated above,
        # so a typo in nodes.yaml fails at launch here too.
        return _DEFAULT_PROFILE[ZENOH_BRIDGE]
    stem = fastdds_filename[len('fastdds_'):]
    stem = stem[:stem.rindex('.')]
    return selected + '_' + stem + _PROFILE_SUFFIX[selected]


def profile_path(share_dir, fastdds_filename):
    """Absolute path of the profile, checked so a typo fails at launch."""
    path = os.path.join(share_dir, 'config', profile_filename(fastdds_filename))
    if not os.path.isfile(path):
        raise RuntimeError(f'DDS profile not found: {path}')
    return path


def profile_env(share_dir, fastdds_filename):
    """Environment that pins one process to the wired IVN profile."""
    path = profile_path(share_dir, fastdds_filename)
    selected = middleware()
    environment = {'RMW_IMPLEMENTATION': _RMW_IMPLEMENTATIONS[selected]}
    if state():
        environment['ROS_DOMAIN_ID'] = '0'
        environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SYSTEM_DEFAULT'
        environment['ROS_STATIC_PEERS'] = ''
        # Empty overrides inherited legacy settings without rcl's deprecation warning.
        environment['ROS_LOCALHOST_ONLY'] = ''
    if selected == FASTDDS:
        environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = path
        environment['FASTDDS_DEFAULT_PROFILES_FILE'] = path
    elif selected in (CYCLONEDDS, ZENOH_BRIDGE, ZENOH_BRIDGE_PICO):
        environment['CYCLONEDDS_URI'] = 'file://' + path
    else:
        environment['ZENOH_SESSION_CONFIG_URI'] = path
    return environment


def image_profile_env(share_dir, image_index, participant_id):
    """Environment for one image node, which pins its own participant index."""
    return profile_env(
        share_dir,
        f'fastdds_image_{image_index:02d}_participant{participant_id}.xml')


def default_profile_env(share_dir):
    """Environment for nodes without a pinned participant index."""
    return profile_env(share_dir, _DEFAULT_PROFILE[FASTDDS])
