import redis
r = redis.Redis(host='localhost', port=6379, db=0)

# Create a pubsub object
pubsub = r.pubsub()

pubsub.subscribe('Sai::Robot::Panda::q::1')

print("Subscribed to channel 'Sai::Robot::Panda::q::1'. Waiting for messages...")

# Listen for messages
for message in pubsub.listen():
    # The first message is usually a subscription confirmation message
    # Each message will be a dict containing type, channel, and data keys
    print(message)

