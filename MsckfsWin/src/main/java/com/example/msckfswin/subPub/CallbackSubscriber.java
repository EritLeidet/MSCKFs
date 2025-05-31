package com.example.msckfswin.subPub;

import java.util.concurrent.Flow.Subscription;
import java.util.concurrent.Flow.Subscriber;
import java.util.function.Consumer;
public class CallbackSubscriber<T> implements Subscriber<T> {

    private final Consumer<T> callback;
    private Subscription subscription;

    public CallbackSubscriber(Consumer<T> callback) {
        this.callback = callback;
    }

    @Override
    public void onSubscribe(Subscription subscription) {
        this.subscription = subscription;
        subscription.request(1);

    }

    @Override
    public void onNext(T t) {
        callback.accept(t);
        subscription.request(1); // new request after blocking call
    }

    @Override
    public void onError(Throwable throwable) {
        throwable.printStackTrace(); //TODO: better logging

    }

    @Override
    public void onComplete() {
        //TODO
    }
}
