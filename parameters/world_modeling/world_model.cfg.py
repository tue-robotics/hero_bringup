import tue_config

cfg = tue_config.from_file("world_model.yaml")
tue_config.write_yaml(cfg)
