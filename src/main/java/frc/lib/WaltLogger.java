package frc.lib;

import edu.wpi.first.networktables.*;

public final class WaltLogger {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable traceTable = inst.getTable("Trace");

    public static DoublePublisher makeDoublePub(NetworkTable table, String name, double period) {
        DoubleTopic topic = table.getDoubleTopic(name);
        return topic.publish(PubSubOption.periodic(period));
    }

    public static DoublePublisher makeDoubleTracePub(String name) {
        return makeDoublePub(traceTable, name, 0.02);
    }

    public static DoubleArrayPublisher makeDoubleArrPub(NetworkTable table, String name, double period) {
        DoubleArrayTopic topic = table.getDoubleArrayTopic(name);
        return topic.publish(PubSubOption.periodic(period));
    }

    public static DoubleArrayPublisher makeDoubleArrTracePub(String name) {
        return makeDoubleArrPub(traceTable, name, 0.02);
    }

    public static BooleanPublisher makeBoolPub(NetworkTable table, String name, double period) {
        BooleanTopic topic = table.getBooleanTopic(name);
        return topic.publish(PubSubOption.periodic(period));
    }

    public static BooleanPublisher makeBoolTracePub(String name) {
        return makeBoolPub(traceTable, name, 0.02);
    }

    public static StringPublisher makeStringPub(NetworkTable table, String name, double period) {
        StringTopic topic = table.getStringTopic(name);
        return topic.publish(PubSubOption.periodic(period));
    }

    public static StringPublisher makeStringTracePub(String name) {
        return makeStringPub(traceTable, name, 0.02);
    }
}
