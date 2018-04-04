/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using WebSocketSharp;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Text;

namespace RosSharp.RosBridgeClient
{
	public class RosSocketEventArgs : EventArgs
	{
		public readonly object Sender;
		public readonly Uri Url;
		public readonly EventArgs Args;
		public RosSocketEventArgs (object sender, Uri url, EventArgs args) {
			this.Sender = sender;
			this.Url = url;
			this.Args = args;
		}
	}

	public delegate void RosSocketEventHandler (object sender, RosSocketEventArgs e);

    public class RosSocket
    {
		#region Public Events
		public event RosSocketEventHandler OnOpen;
		public event RosSocketEventHandler OnClose;
		public event RosSocketEventHandler OnError;
        #endregion

        private RosSharedData sharedData;
        private RosThread producer;
        private Thread producerThread;

        #region Public
        public RosSocket(string url)
        {
            sharedData = new RosSharedData();
            producer = new RosThread(sharedData);
            producerThread = new Thread(new ThreadStart(producer.Process));
            producerThread.Start();

            webSocket = new WebSocket(url);
            webSocket.OnMessage += (sender, e) => recievedOperation((WebSocket)sender, e);
			webSocket.OnError += (sender, e) => {
				if (OnError != null) {
					OnError (this, new RosSocketEventArgs (sender, ((WebSocket)sender).Url, e));
				}
			};
			webSocket.OnOpen += (sender, e) => {
				if (OnOpen != null) {
					OnOpen (this, new RosSocketEventArgs (sender, ((WebSocket)sender).Url, e));
				}
			};
			webSocket.OnClose += (sender, e) => {
				if (OnClose != null) {
					OnClose (this, new RosSocketEventArgs (sender, ((WebSocket)sender).Url, e));
				}
			};
		}

		public void Connect ()
		{
            webSocket.Connect();
        }

        public void Close()
        {
            while (publishers.Count > 0)
                Unadvertize(publishers.First().Key);

            lock (sharedData.SubscribersLock)
            {
                while (sharedData.SubscribersCount > 0)
                    Unsubscribe(sharedData.FirstSubscriberKey);
            }

            producer.IsAlive = false;
            webSocket.Close();
        }

        public int Advertize(string topic, string type)
        {
            int id = generateId();
            publishers.Add(id, new Publisher(topic));

            sendOperation(new Adverisement(id, topic, type));
            return id;
        }

        public void Publish(int id, Message msg)
        {
            Publisher publisher;
            if (publishers.TryGetValue(id, out publisher))
                sendOperation(new Publication(id, publisher.Topic, msg));
        }

        public void Unadvertize(int id)
        {
            sendOperation(new Unadverisement(id, publishers[id].Topic));
            publishers.Remove(id);
        }

        public int Subscribe(string topic, string rosMessageType, RosSharedData.MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Type messageType = MessageTypes.MessageType(rosMessageType);
            if (messageType==null)
                return 0;

            int id = generateId();
            sharedData.AddSubscriber(id, topic, messageType, messageHandler);
            sendOperation(new Subscription(id, topic, rosMessageType, throttle_rate, queue_length, fragment_size, compression));
            return id;

        }

        public int Subscribe(string topic, Type messageType, RosSharedData.MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            string rosMessageType = MessageTypes.RosMessageType(messageType);
            if (rosMessageType == null)
                return 0;

            return Subscribe(topic, messageType, messageHandler, throttle_rate, queue_length, fragment_size, compression);
        }

        public void Unsubscribe(int id)
        {
            string topic = sharedData.RemoveSubscriber(id);
            sendOperation(new Unsubscription(id, topic));
        }

        public int CallService(string service, Type objectType, RosSharedData.ServiceHandler serviceHandler, object args = null)
        {
            int id = generateId();
            sharedData.AddServiceCaller(id, service, objectType, serviceHandler);

            sendOperation(new ServiceCall(id, service, args));
            return id;
        }
        #endregion

        #region Private

        internal struct Publisher
        {
            internal string Topic;
            internal Publisher(string topic)
            {
                Topic = topic;
            }
        }

        private WebSocket webSocket;
        private Dictionary<int, Publisher> publishers = new Dictionary<int, Publisher>();

        private void recievedOperation(object sender, WebSocketSharp.MessageEventArgs e)
        {
			byte[] rawData = e.RawData;
			string data = e.Data.Replace ("null", "0");
			rawData = Encoding.ASCII.GetBytes (data);
            JObject operation = Deserialize(rawData);

#if DEBUG
            Console.WriteLine("Recieved " + operation.GetOperation());
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif
            lock (sharedData.ReceiveLock)
            {
                sharedData.ReceiveQueue.Enqueue(operation);
            }
        }

        private void sendOperation(Operation operation)
        {
#if DEBUG
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif
            webSocket.SendAsync(Serialize(operation), null);
        }
        public static byte[] Serialize(object obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            byte[] buffer = Encoding.ASCII.GetBytes(json);
            int I = json.Length;
            int J = buffer.Length;
            return buffer;
        }

        public static JObject Deserialize(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            int I = ascii.Length;
            int J = buffer.Length;
            return JsonConvert.DeserializeObject<JObject>(ascii);
        }

        private static int generateId()
        {
            return Guid.NewGuid().GetHashCode();
        }
        #endregion       
    }
}
