# Copyright (C) 2021 Rooholla Khorram Bakht, Eren Allak,
# and others, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <r.khorrambakht@alumni.kntu.ac.ir>,
# <eren.allak@aau.at>

import cmake_example as m


def test_main():
    assert m.__version__ == "0.0.1"
    assert m.add(1, 2) == 3
    assert m.subtract(1, 2) == -1
