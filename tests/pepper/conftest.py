def pytest_addoption(parser):
    parser.addoption(
        '--virtualenv',
        metavar='virtualenv',
        default='1',
        help="1 if the tests are on a simulated robot, 0 otherwise")