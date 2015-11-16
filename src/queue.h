typedef void QueueCallback(void *data);

struct WorkEntry {
  QueueCallback *callback;
  void *data;
};

struct Queue {
  u32 volatile next_entry_to_write;
  u32 volatile next_entry_to_read;

  u32 volatile completion_count;
  u32 volatile completion_goal;

  u32 capacity;
  u32 worker_count;

  WorkEntry *entries;

  SDL_sem *semaphore;
};

bool do_queue_work(Queue *queue) {
  bool sleep = false;

  u32 original_next_index = queue->next_entry_to_read;
  int new_next_index = (original_next_index + 1) % queue->capacity;

  if (original_next_index != queue->next_entry_to_write) {
    SDL_bool value = SDL_AtomicCAS((SDL_atomic_t *)&queue->next_entry_to_read, original_next_index, new_next_index);

    if (value) {
      WorkEntry *entry = queue->entries + original_next_index;
      entry->callback(entry->data);

      SDL_AtomicIncRef((SDL_atomic_t *)&queue->completion_count);
    }
  } else {
    sleep = true;
  }

  return sleep;
}

static int thread_function(void *data) {
  Queue *queue = (Queue *)data;

  while (true) {
    if (do_queue_work(queue)) {
      SDL_SemWait(queue->semaphore);
    }
  }
}

void create_workers(Queue *queue, u32 worker_count) {
  queue->worker_count = worker_count;
  for (u32 i=0; i<worker_count; i++) {
    SDL_CreateThread(thread_function, "main_worker_thread", queue);
  }
}

void add_work(Queue *queue, QueueCallback *callback, void *data) {
  u32 new_next_entry_to_write = (queue->next_entry_to_write + 1) % queue->capacity;

  assert(new_next_entry_to_write != queue->next_entry_to_read);

  WorkEntry *entry = queue->entries + queue->next_entry_to_write;

  entry->callback = callback;
  entry->data = data;

  queue->completion_goal += 1;

  SDL_CompilerBarrier();

  queue->next_entry_to_write = new_next_entry_to_write;
  SDL_SemPost(queue->semaphore);
}

void initialize_queue(Queue *queue, u32 capacity) {
  queue->next_entry_to_write = 0;
  queue->next_entry_to_read = 0;

  queue->completion_count = 0;
  queue->completion_goal = 0;
  queue->semaphore = SDL_CreateSemaphore(0);
  queue->entries = (WorkEntry *)malloc(sizeof(WorkEntry) * capacity);
  queue->capacity = capacity;
}

void destroy_queue(Queue *queue) {
  free(queue->entries);
  SDL_DestroySemaphore(queue->semaphore);
}

void complete_all_work(Queue *queue) {
  while (queue->completion_goal != queue->completion_count) {
    do_queue_work(queue);
  }

  queue->completion_count = 0;
  queue->completion_goal = 0;
}
