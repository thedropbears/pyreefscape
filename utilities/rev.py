import rev


def configure_spark_ephemeral(motor: rev.SparkBase, config: rev.SparkBaseConfig):
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kNoResetSafeParameters,
        rev.SparkBase.PersistMode.kNoPersistParameters,
    )


def configure_spark_reset_and_persist(
    motor: rev.SparkBase, config: rev.SparkBaseConfig
):
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kResetSafeParameters,
        rev.SparkBase.PersistMode.kPersistParameters,
    )
