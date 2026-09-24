from wiz import __version__


def get_status() -> dict:
    return {"name": "wiz", "version": __version__, "ready": True}


def main() -> None:
    status = get_status()
    state = "READY" if status["ready"] else "NOT READY"
    print(f"Wiz v{status['version']} - status: {state}")


if __name__ == "__main__":
    main()
