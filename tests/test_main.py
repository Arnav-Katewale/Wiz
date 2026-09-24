from wiz.main import get_status


def test_get_status_shape():
    status = get_status()
    assert status["name"] == "wiz"
    assert "version" in status


def test_get_status_ready():
    assert get_status()["ready"] is True
