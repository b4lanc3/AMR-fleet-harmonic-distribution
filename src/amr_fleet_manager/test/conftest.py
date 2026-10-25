import os

import pytest

from amr_fleet_manager.core.layout import build_graph, load_layout

HERE = os.path.dirname(os.path.abspath(__file__))


def _layout_path() -> str:
    path = os.path.join(HERE, '..', '..', 'amr_bringup', 'config', 'factory_layout.yaml')
    if os.path.exists(path):
        return path
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('amr_bringup'), 'config',
                        'factory_layout.yaml')


@pytest.fixture(scope='session')
def layout():
    return load_layout(_layout_path())


@pytest.fixture(scope='session')
def graph(layout):
    return build_graph(layout)
