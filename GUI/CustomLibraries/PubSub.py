# Define a Topic class
class Topic:
    def __init__(self, topicName = str):
        self.name = topicName
        self.subscribers = []

    def addSubscriber(self, subscriber):
        self.subscribers.append(subscriber)

    def publishToSubs(self, msg):
        print(f"[Topic: {self.name}] Message Sent: {msg}")
        for subscriber in self.subscribers:
            subscriber.receive(msg)

# Publisher class sends messages to a topic
class Publisher:
    def __init__(self, topic: Topic):
        self.topic = topic

    def publishMsg(self, msg):
        self.topic.publishToSubs(msg)

# Subscriber class receives messages from a topic
class Subscriber:
    def __init__(self, subscriberName = str):
        self.name = subscriberName

    def receive(self, pose):
        print(f"{self.name} Message Recieved: {pose}")



