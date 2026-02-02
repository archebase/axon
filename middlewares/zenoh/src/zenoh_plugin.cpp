// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "zenoh_plugin.hpp"

#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <zenoh.hxx>

namespace zenoh_plugin {

// Global Zenoh session (shared across the plugin)
// Using a raw pointer wrapper to avoid including zenoh.hxx in header
namespace {
struct GlobalSession {
  std::shared_ptr<zenoh::Session> session;
  std::unique_ptr<SubscriptionManager> subscription_manager;
  std::mutex mutex;

  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    subscription_manager.reset();
    session.reset();
  }
};

GlobalSession g_global_session;
}  // namespace

// Helper to get Zenoh session (defined in cpp to avoid header dependency)
namespace {
std::shared_ptr<zenoh::Session> get_global_session() {
  std::lock_guard<std::mutex> lock(g_global_session.mutex);
  return g_global_session.session;
}

void set_global_session(std::shared_ptr<zenoh::Session> session) {
  std::lock_guard<std::mutex> lock(g_global_session.mutex);
  g_global_session.session = session;
}

SubscriptionManager* get_subscription_manager() {
  std::lock_guard<std::mutex> lock(g_global_session.mutex);
  return g_global_session.subscription_manager.get();
}

void set_subscription_manager(std::unique_ptr<SubscriptionManager> manager) {
  std::lock_guard<std::mutex> lock(g_global_session.mutex);
  g_global_session.subscription_manager = std::move(manager);
}

}  // namespace

ZenohPlugin::ZenohPlugin()
    : initialized_(false)
    , running_(false) {}

ZenohPlugin::~ZenohPlugin() {
  stop();
}

bool ZenohPlugin::init(const char* config_json) {
  if (initialized_.load()) {
    std::cerr << "[ZenohPlugin] Already initialized" << std::endl;
    return false;
  }

  try {
    // Parse configuration (minimal for MVP)
    zenoh::Config config = zenoh::Config::create_default();

    if (config_json && std::strlen(config_json) > 0) {
      // For MVP, we'll skip JSON parsing and use defaults
      (void)config_json;
    }

    // Open Zenoh session
    // Session::open() throws ZException on failure, no need to check
    auto session = zenoh::Session::open(std::move(config));

    // Store session globally (but don't set flags yet)
    set_global_session(std::make_shared<zenoh::Session>(std::move(session)));

    // Create subscription manager with raw pointer to session
    auto* session_ptr = get_global_session().get();

    auto subscription_manager = std::make_unique<SubscriptionManager>(session_ptr);
    set_subscription_manager(std::move(subscription_manager));

    // Set flags only after all initialization succeeds
    initialized_.store(true);
    running_.store(true);

    std::cout << "[ZenohPlugin] Initialized successfully" << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "[ZenohPlugin] Initialization failed: " << e.what() << std::endl;
    // Clear session (flags not set, so stop() won't try to clear again)
    g_global_session.clear();
    return false;
  } catch (...) {
    std::cerr << "[ZenohPlugin] Initialization failed: unknown exception" << std::endl;
    // Clear session (flags not set, so stop() won't try to clear again)
    g_global_session.clear();
    return false;
  }
}

bool ZenohPlugin::start() {
  if (!initialized_.load()) {
    std::cerr << "[ZenohPlugin] Cannot start: not initialized" << std::endl;
    return false;
  }

  if (running_.load()) {
    std::cout << "[ZenohPlugin] Already running" << std::endl;
    return true;
  }

  // Zenoh session is already active after init
  running_.store(true);
  std::cout << "[ZenohPlugin] Started" << std::endl;
  return true;
}

bool ZenohPlugin::stop() {
  // Always attempt cleanup, even if not fully initialized
  // This handles the case where init() failed after creating the session
  std::cout << "[ZenohPlugin] Stopping..." << std::endl;

  // Clear global session (includes subscription manager)
  g_global_session.clear();

  initialized_.store(false);
  running_.store(false);

  std::cout << "[ZenohPlugin] Stopped" << std::endl;
  return true;
}

bool ZenohPlugin::subscribe(
  const std::string& keyexpr, const std::string& message_type, MessageCallback callback
) {
  if (!initialized_.load()) {
    std::cerr << "[ZenohPlugin] Cannot subscribe: not initialized" << std::endl;
    return false;
  }

  SubscriptionManager* manager = get_subscription_manager();
  if (!manager) {
    std::cerr << "[ZenohPlugin] Subscription manager not available" << std::endl;
    return false;
  }

  return manager->subscribe(keyexpr, message_type, callback);
}

bool ZenohPlugin::unsubscribe(const std::string& keyexpr) {
  if (!initialized_.load()) {
    std::cerr << "[ZenohPlugin] Cannot unsubscribe: not initialized" << std::endl;
    return false;
  }

  SubscriptionManager* manager = get_subscription_manager();
  if (!manager) {
    return false;
  }

  return manager->unsubscribe(keyexpr);
}

std::vector<std::string> ZenohPlugin::get_subscribed_keyexprs() const {
  // Lock first to avoid race with stop()
  std::lock_guard<std::mutex> lock(g_global_session.mutex);

  // Check both initialized flag and subscription_manager inside lock
  if (!initialized_.load() || !g_global_session.subscription_manager) {
    return {};
  }

  return g_global_session.subscription_manager->get_subscribed_keyexprs();
}

}  // namespace zenoh_plugin
