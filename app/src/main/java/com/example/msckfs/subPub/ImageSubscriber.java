package com.example.msckfs.subPub;

import java.util.concurrent.Flow.Subscription;
import java.util.concurrent.Flow.Subscriber;
import java.util.function.Consumer;
public class ImageSubscriber<T> implements Subscriber<T> {

    private final Consumer<T> callback;
    private Subscription subscription;

    public ImageSubscriber(Consumer<T> callback) {
        this.callback = callback;
    }

    @Override
    public void onSubscribe(Subscription subscription) {
        this.subscription = subscription;
        subscription.request(1);

    }

    @Override
    public void onNext(T t) {
        //TODO: request at the end or before the end of method? I've seen before.
        callback.accept(t);
        subscription.request(1);
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
