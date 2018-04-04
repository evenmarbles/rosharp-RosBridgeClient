using System;
using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient
{
    public class RosThread
    {
        public bool IsAlive;

        RosSharedData sharedData;
        JObject operation;

        public RosThread(RosSharedData pointer)
        {
            sharedData = pointer;
            IsAlive = true;
        }

        public void Process()
        {
            while (IsAlive)
            {
                lock (sharedData.ReceiveLock)
                {
                    if (sharedData.ReceiveQueue.Count > 0)
                    {
                        operation = sharedData.ReceiveQueue.Dequeue();
                    }
                }

                if (operation != null)
                {
                    switch (operation.GetOperation())
                    {
                        case "publish":
                            {
                                sharedData.RecievedPublish(operation);
                                break;
                            }
                        case "service_response":
                            {
                                sharedData.RecievedServiceResponse(operation);
                                break;
                            }
                    }
                    operation = null;
                }
            }
        }
    }
}