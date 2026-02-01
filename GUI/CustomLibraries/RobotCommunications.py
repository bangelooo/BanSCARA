import time

# Define a Topic class
class Topic:
    def __init__(self, topicName = str):
        self.name = topicName
        self.subscribers = []

    def addSubscriber(self, subscriber):
        self.subscribers.append(subscriber)

    def publishToSubs(self, msg):
        #print(f"[Topic: {self.name}] Message Sent: {msg}")
        for subscriber in self.subscribers:
            subscriber.receive(msg)

# Publisher class sends messages to a topic
class Publisher:
    def __init__(self, topic: Topic,publishRate: float = 1.0):
        self.topic = topic
        self.publishRate = publishRate # In Hz
        self._lastPubTime = time.monotonic()

    def publishMsg(self, msg):
        if self.publishRate <= 0.0:
            return

        now = time.monotonic()
        rate = 1 / self.publishRate

        if (now - self._lastPubTime) >= rate: 
            self.topic.publishToSubs(msg)
            self._lastPubTime += rate

# Subscriber class receives messages from a topic
class Subscriber:
    def __init__(self, subscriberName = str):
        self.name = subscriberName
        self.msg = None

    def receive(self, msg):
        print(f"[Subscriber: {self.name}] Message Received: {msg}")
        #print(msgString)
        self.msg = msg



