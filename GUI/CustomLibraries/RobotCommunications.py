import time
from enum import Enum
# ======================================
#   Publishers and Subscribers
# ======================================
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
        #print(f"[Subscriber: {self.name}] Message Received: {msg}")
        self.msg = msg

# ======================================
#   Client and Servers
# ======================================
import threading

class ServerState(Enum):
    READY = 1
    BUSY = 2
    CANCELLING = 3

class Service:
    def __init__(self, serviceName:str):
        self.name = serviceName
        self.serviceQueue = []
    
    def processRequest(self,requestMsg,responseCB):
        self.serviceQueue.append((requestMsg,responseCB))


class ServiceClient:
    def __init__(self,service: Service):
        self.service = service

    def sendRequest(self,requestMsg, responseCB):
        print(f"[Client] Sending Request: {requestMsg}")
        self.service.processRequest(requestMsg, responseCB)
    
    def responseCB(self,response):
        print(f"Server Response: {response}")


class ServiceServer:
    def __init__(self,service: Service):
        self.service = service
        self.serverState = ServerState.READY
        self.workingThread = None

    def handleRequest(self):
        if self.serverState == ServerState.READY:
            # Acknowledge that the request has been received
            cb = self.service.serviceQueue[0][1]
            response = self.sendResponse()
            cb(response)

            # Handle the request
            request = self.service.serviceQueue[0][0]
            print(f"Server's Current Action: {request}")
            time.sleep(2)
            self.service.serviceQueue.pop(0)

    def sendResponse(self):
        str = "Request Acknowledged"
        return str
      
# ======================================
#   Action Client and Servers
# ======================================
import threading

class ServerState(Enum):
    READY = 1
    BUSY = 2

class Action:
    def __init__(self,actionName:str):
        self.name = actionName
        self.goalQueue = []
        self.cancelQueue = []
        self.feedbackTopic = Topic(f"{actionName} Action - Feedback")

    
    def processGoal(self,goalMsg,responseCB):
        self.goalQueue.append((goalMsg,responseCB))
    
    
class ActionClient:
    def __init__(self, action:Action):
        self.action = action
        self.feedbackSub = Subscriber(self.action.name)
        self.action.feedbackTopic.addSubscriber(self.feedbackSub)
        self.goalAccepted = False

    # ===================================================
    #   Methods for goal request and registered callbacks
    # ===================================================   
    def sendGoalRequest(self,goalMsg):
        """
        Sends a goal request to the Action Server.
        Adds the goal and a goal resposne callback to the action goal queue.

        :param goalMsg: Requested goal to be processed by the Action Server.
        """
        self.action.processGoal(goalMsg,self._goalResponseCB)

    def _goalResponseCB(self,response):
        """
        Callback for a goal response by the Action Server.
        If the goal is accepted, register a callback for the goal result (request).
    
        :param response: Goal response from the Action Server.
        """
        print(f"{self.action.name} Server response: {response}")
        if response == "ACK":
            # Set goal accepted flag
            self.goalAccepted = True
            print("Goal Accepted")

            # Register a callback to request a response.
            return self.requestResult
  
    def requestResult(self,result):
        print(result)

    # ===================================================
    #   Methods for cancel request and registered callbacks
    # =================================================== 
    def sendCancelRequest(self,goalID):
        print("Cancel requested by client.")
        self.action.cancelQueue.append(("CANCEL",goalID))

    def _cancelResponseCB(self):
        pass
    
    
class ActionServer:
    def __init__(self,action: Action):
        self.action = action
        self.actionServerState = ServerState.READY
        self.workingThread = None
        self.goalHndlrFcn = None
        self.cancelEvent = threading.Event()

        # Feedback topic
        self.feedbackPub = Publisher(self.action.feedbackTopic)
    
    def assignGHF(self,goalHndlrFcn):
        """
        Assign a request handl
        
        :param self: Description
        :param requestHndlrFcn: Description
        """
        if self.goalHndlrFcn == None:
            self.goalHndlrFcn = goalHndlrFcn

        
    # =============================================
    #   Methods to process and handle goal reqeusts
    # =============================================   
    def handleGoalRequest(self):
        """
        Handles goal requests in the Action goal queue.
        If the the Action Server is in a ready state, the goal will be accepted.
        and the goal request callback from the Action Client is invoked.
        From the goal request callback, a result request callback is returned.
        The Action Server enters a BUSY state and calls the executeGoal method, passing
        in the goal request and the result request callback.
        """
        if self.actionServerState == ServerState.READY and self.action.goalQueue:
            request,responseCB = self.action.goalQueue.pop(0)
            response = "ACK"
            # Send response and recieve a request 
            resultRequestCB = responseCB(response)
            
            # Set server into a busy state
            self.actionServerState = ServerState.BUSY
            print(self.actionServerState)
            self.executeGoal(request,resultRequestCB)
        elif self.action.goalQueue:
            _,responseCB = self.action.goalQueue.pop(0)
            response = "NACK"
            responseCB(response)
        else:
            return

    def executeGoal(self,request,resultRequestCB):
        # Handle Request 
        fcn = self.goalHndlrFcn
        # Start desired function on a thread and register callbacks for goal result and feedback
        self.workingThread = threading.Thread(
            target = fcn,
            args = (
                request,
                self._finishGoal,
                self.publishFeedback,
                resultRequestCB,
                #self.cancelEvent,
                ),
            daemon = False)
        print("Executing the goal...")
        self.workingThread.start()
    
    def _finishGoal(self,resultRequestCB):
        # Update server state to READY
        self.actionServerState = ServerState.READY
        result = "Success! Goal completed"

        # Send the result to the client.
        resultRequestCB(result)
        print(self.actionServerState)

    # =============================================
    #  Feedback Publisher
    # =============================================   

    def publishFeedback(self,msg):
        self.feedbackPub.publishMsg(msg)

    # =============================================
    #   Methods to process and handle cancel reqeusts
    # =============================================   
    def handleCancelRequest(self):
        if self.action.cancelQueue and self.actionServerState == ServerState.BUSY:
            self.action.cancelQueue.pop(0)
            print("Server received cancel request")
            self.cancelEvent.set()
