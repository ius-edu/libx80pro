//
//  MainWindowController.h
//  CheckWriter
//
//  Created by Jesse Riddle on 4/27/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import <Cocoa/Cocoa.h>

@interface MainWindowController : NSWindowController
@property (assign) IBOutlet NSPopUpButton *selectAccount;
@property (assign) IBOutlet NSButton *deleteSelectedAccount;
@property (assign) IBOutlet NSTextField *checkNo;
@property (assign) IBOutlet NSDatePicker *checkDate;
@property (assign) IBOutlet NSComboBox *checkPayee;
@property (assign) IBOutlet NSTextField *checkAmount;
@property (assign) IBOutlet NSTextField *checkLongAmount;
@property (assign) IBOutlet NSTextField *checkMemo;
@property (assign) IBOutlet NSButton *checkPrint;
@property (assign) IBOutlet NSTextField *accountDetails;

@end
