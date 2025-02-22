import rev


def configure_spark_ephemeral(motor: rev.SparkBase, config: rev.SparkBaseConfig):
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kNoResetSafeParameters,
        rev.SparkBase.PersistMode.kNoPersistParameters,
    )
