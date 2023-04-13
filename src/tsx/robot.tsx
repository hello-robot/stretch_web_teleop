import React, { useState } from 'react'
import { RosConnection, TopicListProvider } from 'rosreact'

export const Connect = () => {
    const [trigger, setTrigger] = useState(true);

    return (
        <RosConnection url={"wss://localhost:9090"} autoConnect>
            <TopicListProvider
                    trigger={trigger} 
                    failedCallback={(e) => {console.log(e)}}
            >
            </TopicListProvider>
        </RosConnection>
    );
}