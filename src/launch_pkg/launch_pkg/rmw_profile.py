"""Middleware selection for the wired IVN.

Every node that talks to a ZCU has to be pinned to the wired interface and, for
the image nodes, to a fixed participant index. Fast DDS expresses that in an XML
profile named by FASTRTPS_DEFAULT_PROFILES_FILE; Cyclone DDS expresses the same
thing in its own XML named by CYCLONEDDS_URI. This module is the single place
that knows which of the two is active and how to name its profile, so launch
files and the GUI process manager stay middleware-neutral.

Select the middleware with AUTOSDV_RMW=fastdds|cyclonedds (default fastdds).

Profiles come in pairs whose names differ only by prefix, for example
    fastdds_image_03_participant22.xml  <->  cyclonedds_image_03_participant22.xml
so callers keep referring to the Fast DDS name and this module translates it.
"""

import os

FASTDDS = 'fastdds'
CYCLONEDDS = 'cyclonedds'

_RMW_IMPLEMENTATIONS = {
    FASTDDS: 'rmw_fastrtps_cpp',
    CYCLONEDDS: 'rmw_cyclonedds_cpp',
}

# The shared wired-IVN profile is the one pair that is not a simple rename.
_DEFAULT_PROFILE = {
    FASTDDS: 'fastdds_whitelist.xml',
    CYCLONEDDS: 'cyclonedds_ivn.xml',
}


def middleware():
    """Return the selected middleware, validating AUTOSDV_RMW."""
    selected = os.environ.get('AUTOSDV_RMW', FASTDDS).strip().lower()
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
    return selected + '_' + fastdds_filename[len('fastdds_'):]


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
    if selected == FASTDDS:
        environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = path
        environment['FASTDDS_DEFAULT_PROFILES_FILE'] = path
    else:
        environment['CYCLONEDDS_URI'] = 'file://' + path
    return environment


def image_profile_env(share_dir, image_index, participant_id):
    """Environment for one image node, which pins its own participant index."""
    return profile_env(
        share_dir,
        f'fastdds_image_{image_index:02d}_participant{participant_id}.xml')


def default_profile_env(share_dir):
    """Environment for nodes without a pinned participant index."""
    return profile_env(share_dir, _DEFAULT_PROFILE[FASTDDS])
