using System;
using System.Linq;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient
{
    public class RosSharedData
    {
        public int SubscribersCount { get { return subscribers.Count;  } }
        public int FirstSubscriberKey { get { return subscribers.First().Key; } }

        public readonly Queue<JObject> ReceiveQueue;
        public readonly object ReceiveLock = new object();
        public readonly object SubscribersLock = new object();
        public readonly object ServiceCallersLock = new object();

        public delegate void ServiceHandler(object obj);
        public delegate void MessageHandler(Message message);

        private Dictionary<int, Subscriber> subscribers = new Dictionary<int, Subscriber>();
        private Dictionary<int, ServiceCaller> serviceCallers = new Dictionary<int, ServiceCaller>();

        internal struct Subscriber
        {
            internal string topic;
            internal Type messageType;
            internal MessageHandler messageHandler;
            internal Subscriber(string Topic, Type MessageType, MessageHandler MessageHandler)
            {
                topic = Topic;
                messageType = MessageType;
                messageHandler = MessageHandler;
            }
        }
        internal struct ServiceCaller
        {
            internal string service;
            internal Type objectType;
            internal ServiceHandler serviceHandler;
            internal ServiceCaller(string Service, Type ObjectType, ServiceHandler ServiceHandler)
            {
                service = Service;
                objectType = ObjectType;
                serviceHandler = ServiceHandler;
            }
        }

        public RosSharedData()
        {
            ReceiveQueue = new Queue<JObject>();
        }

        public void AddSubscriber(int id, string topic, Type messageType, MessageHandler messageHandler)
        {
            lock (SubscribersLock)
            {
                subscribers.Add(id, new Subscriber(topic, messageType, messageHandler));
            }
        }

        public string RemoveSubscriber(int id)
        {
            string topic = subscribers[id].topic;
            lock (SubscribersLock)
            {
                subscribers.Remove(id);
            }
            return topic;
        }

        public void AddServiceCaller(int id, string service, Type objectType, ServiceHandler serviceHandler)
        {
            lock (ServiceCallersLock)
            {
                serviceCallers.Add(id, new ServiceCaller(service, objectType, serviceHandler));
            }
        }

        public void RecievedServiceResponse(JObject serviceResponse)
        {
            ServiceCaller serviceCaller;
            lock (ServiceCallersLock)
            {
                bool foundById = serviceCallers.TryGetValue(serviceResponse.GetServiceId(), out serviceCaller);

                if (!foundById)
                    serviceCaller = serviceCallers.Values.FirstOrDefault(x => x.service.Equals(serviceResponse.GetService()));
            }


            JObject jObject = serviceResponse.GetValues();
            Type type = serviceCaller.objectType;
            if (serviceCaller.serviceHandler != null)
            {
                if (type != null)
                    serviceCaller.serviceHandler.Invoke(jObject.ToObject(type));
                else
                    serviceCaller.serviceHandler.Invoke(jObject);
            }
        }

        public void RecievedPublish(JObject publication)
        {
            Subscriber subscriber;
            lock (SubscribersLock)
            {
                bool foundById = subscribers.TryGetValue(publication.GetServiceId(), out subscriber);

                if (!foundById)
                    subscriber = subscribers.Values.FirstOrDefault(x => x.topic.Equals(publication.GetTopic()));
            }

            if (subscriber.messageHandler != null)
            {
                subscriber.messageHandler.Invoke((Message)publication.GetMessage().ToObject(subscriber.messageType));
            }
        }
    }
}